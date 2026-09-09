#include "Neo3Sensors.h"

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr float IST8310_UT_PER_LSB = 0.30f;
// IST8310 full-scale from the vendor-compatible ArduPilot/PX4 driver family.
// Reject impossible raw values before they can contaminate heading fusion.
constexpr int16_t IST8310_MAX_RAW_XY = 5334;  // ~1600 uT / 0.3 uT/LSB
constexpr int16_t IST8310_MAX_RAW_Z  = 8334;  // ~2500 uT / 0.3 uT/LSB

inline void ubxChecksumAdd(uint8_t byte, uint8_t &a, uint8_t &b) {
  a = static_cast<uint8_t>(a + byte);
  b = static_cast<uint8_t>(b + a);
}


// Native fixed-capacity formatter. It never allocates and emits each sensor record
// as one atomic USB CDC write, keeping VESC traffic deterministic.
class CdcLineBuffer {
 public:
  CdcLineBuffer(char *buffer, std::size_t capacity) : buffer_(buffer), capacity_(capacity) {}
  void print(const char *v) { append("%s", v == nullptr ? "" : v); }
  void print(char v) {
    if (length_ < capacity_) buffer_[length_++] = v;
    else overflow_ = true;
  }
  void print(int v) { append("%d", v); }
  void print(unsigned int v) { append("%u", v); }
  void print(long v) { append("%ld", v); }
  void print(unsigned long v) { append("%lu", v); }
  void print(float v, int digits = 2) { appendFloat(static_cast<double>(v), digits); }
  void print(double v, int digits = 2) { appendFloat(v, digits); }
  void println(int v) { print(v); endLine(); }
  void println(unsigned int v) { print(v); endLine(); }
  void println(long v) { print(v); endLine(); }
  void println(unsigned long v) { print(v); endLine(); }
  void println(float v, int digits = 2) { print(v, digits); endLine(); }
  void println(double v, int digits = 2) { print(v, digits); endLine(); }
  bool flushToUsb() {
    if (overflow_ || length_ == 0U) return false;
    if (gUsb.availableForWrite() < static_cast<int>(length_)) return false;
    return gUsb.write(reinterpret_cast<const uint8_t *>(buffer_), length_) == length_;
  }
 private:
  void endLine() { print('\r'); print('\n'); }
  void appendFloat(double number, int digits) {
    if (digits < 0) digits = 2;
    if (std::isnan(number)) { print("nan"); return; }
    if (std::isinf(number)) { print("inf"); return; }
    if (number > 4294967040.0 || number < -4294967040.0) { print("ovf"); return; }
    if (number < 0.0) { print('-'); number = -number; }
    double rounding = 0.5;
    for (int i = 0; i < digits; ++i) rounding /= 10.0;
    number += rounding;
    const unsigned long integer = static_cast<unsigned long>(number);
    double remainder = number - static_cast<double>(integer);
    print(integer);
    if (digits > 0) print('.');
    while (digits-- > 0) {
      remainder *= 10.0;
      const unsigned int digit = static_cast<unsigned int>(remainder);
      print(digit);
      remainder -= static_cast<double>(digit);
    }
  }
  void append(const char *format, ...) {
    if (overflow_ || length_ >= capacity_) { overflow_ = true; return; }
    va_list args;
    va_start(args, format);
    const int n = std::vsnprintf(buffer_ + length_, capacity_ - length_, format, args);
    va_end(args);
    if (n < 0 || static_cast<std::size_t>(n) >= capacity_ - length_) {
      overflow_ = true;
      return;
    }
    length_ += static_cast<std::size_t>(n);
  }
  char *buffer_;
  std::size_t capacity_;
  std::size_t length_{0U};
  bool overflow_{false};
};}

void Neo3Sensors::begin() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // safety LED active-low: off
  Board_BuzzerStop();
  gnss_uart_ok_ = gGnssUart.begin(GNSS_BAUD);
  last_gnss_uart_retry_ms_ = HAL_GetTick();
  HAL_Delay(5U);
  ist_ok_ = initIst8310();
  config_attempts_ = 0U;
  if (gnss_uart_ok_ && configureGnss()) config_attempts_ = 1U;
  last_config_ms_ = HAL_GetTick();
  switch_raw_ = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET;
  switch_pressed_ = switch_raw_;
  switch_changed_ms_ = HAL_GetTick();
  updateSafetyLed();
  publishHardwareStatus(true);
}

void Neo3Sensors::pollSafetyIo() {
  pollSafetySwitch();
  updateSafetyLed();
}

void Neo3Sensors::poll() {
  pollGnss();
  pollIst8310();
  pollSafetyIo();

  const uint32_t now_ms = HAL_GetTick();
  if (!gnss_uart_ok_ && static_cast<uint32_t>(now_ms - last_gnss_uart_retry_ms_) >= GNSS_UART_RETRY_MS) {
    last_gnss_uart_retry_ms_ = now_ms;
    gGnssUart.end();
    gnss_uart_ok_ = gGnssUart.begin(GNSS_BAUD);
    if (!gnss_uart_ok_) ++gnss_uart_error_count_;
    else if (configureGnss() && config_attempts_ < 255U) ++config_attempts_;
  }
  // Configuration recovery follows transport freshness, not GNSS fix validity.
  // Indoors NAV-PVT can stream correctly at 10 Hz with fix_type=0/LLH invalid;
  // repeatedly VALSET-configuring a healthy receiver in that state is needless.
  const bool pvt_stream_fresh = pvt_.received_ms != 0U &&
    static_cast<uint32_t>(now_ms - pvt_.received_ms) <= PVT_STALE_MS;
  if (!pvt_stream_fresh) {
    // RAM-only UBX configuration is idempotent. Retry quickly at boot, then
    // periodically so a receiver power-cycle recovers without resetting HMI.
    const uint32_t retry_ms = config_attempts_ < 3 ? GNSS_CONFIG_RETRY_MS : 10000UL;
    if (gnss_uart_ok_ && static_cast<uint32_t>(now_ms - last_config_ms_) >= retry_ms) {
      if (configureGnss() && config_attempts_ < 255U) ++config_attempts_;
      last_config_ms_ = now_ms;
    }
    if (nmea_.gga_valid && static_cast<uint32_t>(now_ms - nmea_.gga_ms) <= NMEA_FRESH_MS) {
      static uint32_t last_fallback_publish_ms = 0;
      if (static_cast<uint32_t>(now_ms - last_fallback_publish_ms) >= 200) {
        last_fallback_publish_ms = now_ms;
        publishNmeaFallback();
      }
    }
  }

  if (static_cast<uint32_t>(now_ms - last_hw_publish_ms_) >= HW_PUBLISH_MS) {
    publishHardwareStatus();
  }
}

bool Neo3Sensors::handleHostCommand(const char *command) {
  if (command == nullptr || strncmp(command, "NEO:", 4) != 0) return false;

  if (strcmp(command, "NEO:LED:AUTO") == 0) {
    led_mode_ = LedMode::AUTO;
    updateSafetyLed();
    (void)gUsb.writeLine("ACK:NEO:LED:AUTO");
    return true;
  }
  if (strcmp(command, "NEO:LED:ON") == 0) {
    led_mode_ = LedMode::FORCE_ON;
    updateSafetyLed();
    (void)gUsb.writeLine("ACK:NEO:LED:ON");
    return true;
  }
  if (strcmp(command, "NEO:LED:OFF") == 0) {
    led_mode_ = LedMode::FORCE_OFF;
    updateSafetyLed();
    (void)gUsb.writeLine("ACK:NEO:LED:OFF");
    return true;
  }
  if (strcmp(command, "NEO:BUZZER:OFF") == 0) {
    Board_BuzzerStop();
    (void)gUsb.writeLine("ACK:NEO:BUZZER:OFF");
    return true;
  }
  if (strcmp(command, "NEO:STATUS") == 0) {
    publishHardwareStatus(true);
    return true;
  }
  if (strncmp(command, "NEO:BEEP:", 9) == 0) {
    unsigned int frequency = 0;
    unsigned int duration = 0;
    if (sscanf(command + 9, "%u:%u", &frequency, &duration) == 2) {
      frequency = std::clamp(frequency, 200U, 5000U);
      duration = std::clamp(duration, 20U, 2000U);
      beep(static_cast<uint16_t>(frequency), static_cast<uint16_t>(duration));
      (void)gUsb.writeLine("ACK:NEO:BEEP");
    } else {
      (void)gUsb.writeLine("ERR:NEO:BEEP_FORMAT");
    }
    return true;
  }

  (void)gUsb.writeLine("ERR:NEO:UNKNOWN_COMMAND");
  return true;
}

void Neo3Sensors::pollGnss() {
  uint16_t budget = 768;
  while (budget-- > 0 && gGnssUart.available() > 0) {
    const int value = gGnssUart.read();
    if (value < 0) break;
    consumeGnssByte(static_cast<uint8_t>(value));
  }
}

void Neo3Sensors::consumeGnssByte(uint8_t b) {
  consumeUbxByte(b);
  consumeNmeaByte(static_cast<char>(b));
}

void Neo3Sensors::resetUbx() {
  ubx_state_ = UbxState::SYNC1;
  ubx_class_ = 0;
  ubx_id_ = 0;
  ubx_length_ = 0;
  ubx_pos_ = 0;
  ubx_ck_a_ = ubx_ck_b_ = 0;
  ubx_rx_ck_a_ = 0;
}

void Neo3Sensors::consumeUbxByte(uint8_t b) {
  switch (ubx_state_) {
    case UbxState::SYNC1:
      if (b == 0xB5) ubx_state_ = UbxState::SYNC2;
      break;
    case UbxState::SYNC2:
      if (b == 0x62) {
        ubx_state_ = UbxState::CLASS;
        ubx_ck_a_ = ubx_ck_b_ = 0;
      } else {
        ubx_state_ = b == 0xB5 ? UbxState::SYNC2 : UbxState::SYNC1;
      }
      break;
    case UbxState::CLASS:
      ubx_class_ = b;
      ubxChecksumAdd(b, ubx_ck_a_, ubx_ck_b_);
      ubx_state_ = UbxState::ID;
      break;
    case UbxState::ID:
      ubx_id_ = b;
      ubxChecksumAdd(b, ubx_ck_a_, ubx_ck_b_);
      ubx_state_ = UbxState::LEN1;
      break;
    case UbxState::LEN1:
      ubx_length_ = b;
      ubxChecksumAdd(b, ubx_ck_a_, ubx_ck_b_);
      ubx_state_ = UbxState::LEN2;
      break;
    case UbxState::LEN2:
      ubx_length_ |= static_cast<uint16_t>(b) << 8;
      ubxChecksumAdd(b, ubx_ck_a_, ubx_ck_b_);
      ubx_pos_ = 0;
      if (ubx_length_ > UBX_MAX_PAYLOAD) {
        resetUbx();
      } else {
        ubx_state_ = ubx_length_ == 0 ? UbxState::CKA : UbxState::PAYLOAD;
      }
      break;
    case UbxState::PAYLOAD:
      ubx_payload_[ubx_pos_++] = b;
      ubxChecksumAdd(b, ubx_ck_a_, ubx_ck_b_);
      if (ubx_pos_ >= ubx_length_) ubx_state_ = UbxState::CKA;
      break;
    case UbxState::CKA:
      ubx_rx_ck_a_ = b;
      ubx_state_ = UbxState::CKB;
      break;
    case UbxState::CKB:
      if (ubx_rx_ck_a_ == ubx_ck_a_ && b == ubx_ck_b_) handleUbxFrame();
      resetUbx();
      break;
  }
}

void Neo3Sensors::handleUbxFrame() {
  if (ubx_class_ == 0x01 && ubx_id_ == 0x07 && ubx_length_ >= 92) {
    parseNavPvt();
  }
}

void Neo3Sensors::parseNavPvt() {
  PVT next;
  next.itow_ms = readU32LE(&ubx_payload_[0]);
  next.fix_type = ubx_payload_[20];
  const uint8_t flags = ubx_payload_[21];
  next.flags2 = ubx_payload_[22];
  next.satellites = ubx_payload_[23];
  next.flags3 = readU16LE(&ubx_payload_[78]);
  next.fix_ok = (flags & 0x01U) != 0U;
  next.invalid_llh = (next.flags3 & 0x0001U) != 0U;

  next.longitude = static_cast<double>(readI32LE(&ubx_payload_[24])) * 1.0e-7;
  next.latitude = static_cast<double>(readI32LE(&ubx_payload_[28])) * 1.0e-7;
  next.altitude_m = static_cast<float>(readI32LE(&ubx_payload_[36])) * 0.001f;
  next.hacc_m = static_cast<float>(readU32LE(&ubx_payload_[40])) * 0.001f;
  next.vacc_m = static_cast<float>(readU32LE(&ubx_payload_[44])) * 0.001f;
  next.vel_n_mps = static_cast<float>(readI32LE(&ubx_payload_[48])) * 0.001f;
  next.vel_e_mps = static_cast<float>(readI32LE(&ubx_payload_[52])) * 0.001f;
  next.vel_d_mps = static_cast<float>(readI32LE(&ubx_payload_[56])) * 0.001f;
  next.ground_speed_mps = static_cast<float>(readI32LE(&ubx_payload_[60])) * 0.001f;
  next.course_deg_ned = static_cast<float>(readI32LE(&ubx_payload_[64])) * 1.0e-5f;
  next.sacc_mps = static_cast<float>(readU32LE(&ubx_payload_[68])) * 0.001f;
  next.course_acc_deg = static_cast<float>(readU32LE(&ubx_payload_[72])) * 1.0e-5f;
  next.pdop = static_cast<float>(readU16LE(&ubx_payload_[76])) * 0.01f;
  next.received_ms = HAL_GetTick();

  const bool coordinates_ok = std::isfinite(next.latitude) && std::isfinite(next.longitude) &&
    std::fabs(next.latitude) <= 90.0 && std::fabs(next.longitude) <= 180.0 &&
    !(std::fabs(next.latitude) < 1.0e-12 && std::fabs(next.longitude) < 1.0e-12);
  next.valid = coordinates_ok;

  if (have_last_pvt_itow_) {
    int32_t delta_ms = static_cast<int32_t>(next.itow_ms - last_pvt_itow_);
    if (delta_ms < -302400000L) delta_ms += 604800000L;
    if (delta_ms > 0 && delta_ms <= 5000) {
      const float instant_hz = 1000.0f / static_cast<float>(delta_ms);
      next.rate_hz = pvt_.rate_hz > 0.0f ? 0.90f * pvt_.rate_hz + 0.10f * instant_hz : instant_hz;
    } else {
      next.rate_hz = pvt_.rate_hz;
    }
  }
  have_last_pvt_itow_ = true;
  last_pvt_itow_ = next.itow_ms;
  pvt_ = next;
  publishPvt();
}

void Neo3Sensors::publishPvt() {
  ++gnss_sequence_;
  char line[320]; CdcLineBuffer out(line, sizeof(line));
  out.print("SENS:GNSS:");
  out.print(gnss_sequence_); out.print(',');
  out.print(pvt_.received_ms); out.print(',');
  out.print(pvt_.itow_ms); out.print(',');
  out.print(pvt_.fix_type); out.print(',');
  out.print(pvt_.fix_ok ? 1 : 0); out.print(',');
  out.print(pvt_.invalid_llh ? 1 : 0); out.print(',');
  out.print(pvt_.satellites); out.print(',');
  out.print(pvt_.latitude, 7); out.print(',');
  out.print(pvt_.longitude, 7); out.print(',');
  out.print(pvt_.altitude_m, 3); out.print(',');
  out.print(pvt_.hacc_m, 3); out.print(',');
  out.print(pvt_.vacc_m, 3); out.print(',');
  out.print(pvt_.vel_n_mps, 4); out.print(',');
  out.print(pvt_.vel_e_mps, 4); out.print(',');
  out.print(pvt_.vel_d_mps, 4); out.print(',');
  out.print(pvt_.ground_speed_mps, 4); out.print(',');
  out.print(normalize360(pvt_.course_deg_ned), 3); out.print(',');
  out.print(pvt_.sacc_mps, 4); out.print(',');
  out.print(pvt_.course_acc_deg, 3); out.print(',');
  out.print(pvt_.pdop, 2); out.print(',');
  out.print(pvt_.rate_hz, 2); out.print(',');
  out.print(pvt_.flags2); out.print(',');
  out.println(pvt_.flags3);
  (void)out.flushToUsb();
}

void Neo3Sensors::consumeNmeaByte(char c) {
  if (c == '$') {
    nmea_collecting_ = true;
    nmea_len_ = 0;
    nmea_line_[nmea_len_++] = c;
    return;
  }
  if (!nmea_collecting_) return;
  if (c == '\r') return;
  if (c == '\n') {
    nmea_line_[nmea_len_] = '\0';
    if (nmea_len_ > 6 && validateNmeaChecksum(nmea_line_)) parseNmeaSentence(nmea_line_);
    nmea_collecting_ = false;
    nmea_len_ = 0;
    return;
  }
  if (nmea_len_ < sizeof(nmea_line_) - 1) {
    nmea_line_[nmea_len_++] = c;
  } else {
    nmea_collecting_ = false;
    nmea_len_ = 0;
  }
}

bool Neo3Sensors::validateNmeaChecksum(const char *sentence) const {
  if (sentence == nullptr || sentence[0] != '$') return false;
  const char *star = strchr(sentence, '*');
  if (star == nullptr || star[1] == '\0' || star[2] == '\0') return false;
  uint8_t calculated = 0;
  for (const char *p = sentence + 1; p < star; ++p) calculated ^= static_cast<uint8_t>(*p);
  char hex[3] = {star[1], star[2], '\0'};
  char *end = nullptr;
  const unsigned long expected = strtoul(hex, &end, 16);
  return end == hex + 2 && expected <= 0xFFUL && calculated == static_cast<uint8_t>(expected);
}

uint8_t Neo3Sensors::splitCsv(char *text, char **fields, uint8_t max_fields) {
  if (text == nullptr || fields == nullptr || max_fields == 0) return 0;
  uint8_t count = 0;
  fields[count++] = text;
  for (char *p = text; *p && count < max_fields; ++p) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }
  return count;
}

bool Neo3Sensors::parseNmeaCoordinate(const char *text, char hemisphere, bool latitude, double &out) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const double raw = strtod(text, &end);
  if (end == text || !std::isfinite(raw) || raw < 0.0) return false;
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees) * 100.0;
  const int max_deg = latitude ? 90 : 180;
  if (degrees < 0 || degrees > max_deg || minutes < 0.0 || minutes >= 60.0) return false;
  if (latitude && hemisphere != 'N' && hemisphere != 'S') return false;
  if (!latitude && hemisphere != 'E' && hemisphere != 'W') return false;
  out = static_cast<double>(degrees) + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') out = -out;
  return std::isfinite(out);
}

void Neo3Sensors::parseNmeaSentence(char *sentence) {
  char *star = strchr(sentence, '*');
  if (star != nullptr) *star = '\0';
  char *fields[20]{};
  const uint8_t count = splitCsv(sentence, fields, 20);
  if (count == 0) return;
  const size_t type_len = strlen(fields[0]);
  const char *type = type_len >= 3 ? fields[0] + type_len - 3 : fields[0];
  const uint32_t now_ms = HAL_GetTick();

  if (strcmp(type, "GGA") == 0 && count >= 10) {
    double lat = 0.0, lon = 0.0;
    const bool coord_ok = fields[3][0] && fields[5][0] &&
      parseNmeaCoordinate(fields[2], fields[3][0], true, lat) &&
      parseNmeaCoordinate(fields[4], fields[5][0], false, lon);
    const int fix = atoi(fields[6]);
    const int sats = atoi(fields[7]);
    const float hdop = static_cast<float>(atof(fields[8]));
    const float alt = static_cast<float>(atof(fields[9]));
    nmea_.gga_valid = coord_ok && fix > 0 && sats >= 0 && std::isfinite(hdop) && hdop > 0.0f;
    nmea_.fix_quality = static_cast<uint8_t>(std::clamp(fix, 0, 9));
    nmea_.satellites = static_cast<uint8_t>(std::clamp(sats, 0, 99));
    nmea_.latitude = lat;
    nmea_.longitude = lon;
    nmea_.altitude_m = std::isfinite(alt) ? alt : 0.0f;
    nmea_.hdop = std::isfinite(hdop) ? hdop : 99.9f;
    nmea_.gga_ms = now_ms;
  } else if (strcmp(type, "RMC") == 0 && count >= 9) {
    const bool active = fields[2][0] == 'A';
    const float knots = static_cast<float>(atof(fields[7]));
    const float course = static_cast<float>(atof(fields[8]));
    nmea_.rmc_valid = active && std::isfinite(knots) && knots >= 0.0f && std::isfinite(course);
    nmea_.speed_mps = nmea_.rmc_valid ? knots * 0.514444f : 0.0f;
    nmea_.course_deg_ned = nmea_.rmc_valid ? normalize360(course) : 0.0f;
    nmea_.rmc_ms = now_ms;
  }
}

void Neo3Sensors::publishNmeaFallback() {
  const uint32_t now_ms = HAL_GetTick();
  const bool rmc_fresh = nmea_.rmc_valid && static_cast<uint32_t>(now_ms - nmea_.rmc_ms) <= 2000;
  ++gnss_sequence_;
  char line[192]; CdcLineBuffer out(line, sizeof(line));
  out.print("SENS:GNSSF:");
  out.print(gnss_sequence_); out.print(',');
  out.print(now_ms); out.print(',');
  out.print(nmea_.fix_quality); out.print(',');
  out.print(nmea_.satellites); out.print(',');
  out.print(nmea_.latitude, 7); out.print(',');
  out.print(nmea_.longitude, 7); out.print(',');
  out.print(nmea_.altitude_m, 3); out.print(',');
  out.print(nmea_.hdop, 2); out.print(',');
  out.print(rmc_fresh ? nmea_.speed_mps : -1.0f, 4); out.print(',');
  out.println(rmc_fresh ? nmea_.course_deg_ned : -1.0f, 3);
  (void)out.flushToUsb();
}

void Neo3Sensors::appendU32(uint8_t *payload, uint16_t &pos, uint32_t value) {
  payload[pos++] = static_cast<uint8_t>(value & 0xFFU);
  payload[pos++] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  payload[pos++] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  payload[pos++] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

void Neo3Sensors::appendU16(uint8_t *payload, uint16_t &pos, uint16_t value) {
  payload[pos++] = static_cast<uint8_t>(value & 0xFFU);
  payload[pos++] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

void Neo3Sensors::appendU8(uint8_t *payload, uint16_t &pos, uint8_t value) {
  payload[pos++] = value;
}

bool Neo3Sensors::sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length) {
  if (!gnss_uart_ok_ || length > UBX_MAX_PAYLOAD || (length > 0U && payload == nullptr)) return false;
  uint8_t frame[UBX_MAX_PAYLOAD + 8U]{};
  uint16_t pos = 0U;
  frame[pos++] = 0xB5U;
  frame[pos++] = 0x62U;
  frame[pos++] = cls;
  frame[pos++] = id;
  frame[pos++] = static_cast<uint8_t>(length & 0xFFU);
  frame[pos++] = static_cast<uint8_t>(length >> 8U);
  uint8_t ck_a = 0U, ck_b = 0U;
  for (uint16_t i = 2U; i < 6U; ++i) ubxChecksumAdd(frame[i], ck_a, ck_b);
  for (uint16_t i = 0U; i < length; ++i) {
    frame[pos++] = payload[i];
    ubxChecksumAdd(payload[i], ck_a, ck_b);
  }
  frame[pos++] = ck_a;
  frame[pos++] = ck_b;
  const std::size_t written = gGnssUart.write(frame, pos);
  if (written != pos) {
    ++gnss_uart_error_count_;
    gnss_uart_ok_ = false;
    return false;
  }
  gGnssUart.flush();
  return true;
}

bool Neo3Sensors::configureGnss() {
  // UBX-CFG-VALSET v0, RAM layer only. Never writes BBR/flash.
  uint8_t payload[160]{};
  uint16_t pos = 0;
  payload[pos++] = 0x00; payload[pos++] = 0x01; payload[pos++] = 0x00; payload[pos++] = 0x00;

  auto logical = [&](uint32_t key, bool enabled) {
    appendU32(payload, pos, key); appendU8(payload, pos, enabled ? 1U : 0U);
  };
  auto u1 = [&](uint32_t key, uint8_t value) {
    appendU32(payload, pos, key); appendU8(payload, pos, value);
  };
  auto u2 = [&](uint32_t key, uint16_t value) {
    appendU32(payload, pos, key); appendU16(payload, pos, value);
  };

  logical(0x10730001UL, true);  // UART1 IN UBX
  logical(0x10750001UL, true);  // UART2 IN UBX
  logical(0x10740001UL, true);  // UART1 OUT UBX
  logical(0x10760001UL, true);  // UART2 OUT UBX
  u1(0x20910007UL, 1U);        // NAV-PVT UART1 every navigation epoch
  u1(0x20910008UL, 1U);        // NAV-PVT UART2 every navigation epoch
  // At 10 Hz, the M9N default NMEA set can saturate 38400 baud. Keep only
  // compact UBX NAV-PVT on both candidate UARTs; configuration is RAM-only.
  const uint32_t nmea_uart_keys[] = {
    0x209100bbUL,0x209100bcUL, // GGA UART1/UART2
    0x209100caUL,0x209100cbUL, // GLL
    0x209100c0UL,0x209100c1UL, // GSA
    0x209100c5UL,0x209100c6UL, // GSV
    0x209100acUL,0x209100adUL, // RMC
    0x209100b1UL,0x209100b2UL  // VTG
  };
  for (uint32_t key : nmea_uart_keys) u1(key, 0U);
  u1(0x20110021UL, 4U);        // automotive dynamic model
  u2(0x30210001UL, 100U);      // measurement period 100 ms = 10 Hz
  u2(0x30210002UL, 1U);        // one nav solution / measurement
  return sendUbx(0x06, 0x8A, payload, pos);
}

bool Neo3Sensors::istWriteAt(uint8_t addr, uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c1, static_cast<uint16_t>(addr) << 1U, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, 5U) == HAL_OK;
}

bool Neo3Sensors::istReadAt(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t count) {
  if (dst == nullptr || count == 0U) return false;
  return HAL_I2C_Mem_Read(&hi2c1, static_cast<uint16_t>(addr) << 1U, reg,
                          I2C_MEMADD_SIZE_8BIT, dst, count, 5U) == HAL_OK;
}

bool Neo3Sensors::istWrite(uint8_t reg, uint8_t value) { return istWriteAt(ist_addr_, reg, value); }
bool Neo3Sensors::istRead(uint8_t reg, uint8_t *dst, uint8_t count) { return istReadAt(ist_addr_, reg, dst, count); }


void Neo3Sensors::recoverIstBus() {
  (void)HAL_I2C_DeInit(&hi2c1);
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  Board_DelayUs(5U);

  // Clock a slave out of a half-finished byte. Stop early once SDA releases.
  for (uint8_t i = 0U; i < 9U && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; ++i) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    Board_DelayUs(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    Board_DelayUs(5U);
  }

  // Generate an explicit I2C STOP (SDA low -> high while SCL is high).
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  Board_DelayUs(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  Board_DelayUs(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  Board_DelayUs(5U);

  Board_ReinitI2c1();
  HAL_Delay(2U);
}

bool Neo3Sensors::initIst8310() {
  recoverIstBus();
  ist_init_error_ = 1;
  ist_whoami_ = 0;
  bool found = false;
  const uint8_t order[] = {ist_addr_, IST8310_ADDR, 0x0C, 0x0D, 0x0F};
  for (uint8_t idx = 0; idx < sizeof(order); ++idx) {
    const uint8_t addr = order[idx];
    if (addr < IST8310_ADDR_MIN || addr > IST8310_ADDR_MAX) continue;
    bool duplicate = false;
    for (uint8_t j = 0; j < idx; ++j) if (order[j] == addr) duplicate = true;
    if (duplicate) continue;
    uint8_t who = 0xFF;
    if (istReadAt(addr, IST8310_WHOAMI_REG, &who, 1) &&
        (who == IST8310_WHOAMI || who == IST8310J_WHOAMI)) {
      ist_addr_ = addr; ist_whoami_ = who; found = true; break;
    }
  }
  if (!found) {
    // Prime a newly powered/hot-plugged sensor for the next retry, but do not
    // block the main loop waiting for POR. GNSS/VESC remain first-class traffic.
    for (uint8_t addr = IST8310_ADDR_MIN; addr <= IST8310_ADDR_MAX; ++addr)
      (void)istWriteAt(addr, IST8310_CTRL2, 0x01);
    return false;
  }
  ist_init_error_ = 2;
  if (!istWrite(IST8310_AVGCNTL, 0x24)) return false;
  if (!istWrite(IST8310_PDCNTL, 0xC0)) return false;
  if (!istWrite(IST8310_CTRL3, 0x00)) return false;
  uint8_t avg = 0, pd = 0, ctrl3 = 0;
  if (!istRead(IST8310_AVGCNTL, &avg, 1) || avg != 0x24 ||
      !istRead(IST8310_PDCNTL, &pd, 1) || pd != 0xC0 ||
      !istRead(IST8310_CTRL3, &ctrl3, 1) || ctrl3 != 0x00) {
    ist_init_error_ = 3; return false;
  }
  ist_init_error_ = 0;
  ist_error_count_ = 0;
  startIstMeasurement();
  return true;
}

void Neo3Sensors::startIstMeasurement() {
  if (istWrite(IST8310_CTRL1, 0x01)) last_ist_measurement_ms_ = HAL_GetTick();
}

void Neo3Sensors::pollIst8310() {
  const uint32_t now_ms = HAL_GetTick();
  if (!ist_ok_) {
    if (static_cast<uint32_t>(now_ms - last_ist_retry_ms_) >= 2000) {
      last_ist_retry_ms_ = now_ms;
      ist_ok_ = initIst8310();
    }
    return;
  }

  // IST8310 single conversion is ~10 ms. Polling status is non-blocking on the
  // CPU; six data bytes are read only when DRDY is asserted.
  if (static_cast<uint32_t>(now_ms - last_ist_measurement_ms_) < 10) return;
  uint8_t status = 0;
  if (!istRead(IST8310_STAT1, &status, 1)) {
    if (++ist_error_count_ >= 5) ist_ok_ = false;
    return;
  }
  if ((status & 0x01U) == 0U) {
    if (static_cast<uint32_t>(now_ms - last_ist_measurement_ms_) > 100) startIstMeasurement();
    return;
  }

  uint8_t raw[6]{};
  if (!istRead(IST8310_DATA_XL, raw, sizeof(raw))) {
    if (++ist_error_count_ >= 5) ist_ok_ = false;
    return;
  }
  ist_error_count_ = 0;
  const int16_t x = static_cast<int16_t>(static_cast<uint16_t>(raw[0]) | (static_cast<uint16_t>(raw[1]) << 8));
  const int16_t y = static_cast<int16_t>(static_cast<uint16_t>(raw[2]) | (static_cast<uint16_t>(raw[3]) << 8));
  const int16_t z_sensor = static_cast<int16_t>(static_cast<uint16_t>(raw[4]) | (static_cast<uint16_t>(raw[5]) << 8));
  if (x > IST8310_MAX_RAW_XY || x < -IST8310_MAX_RAW_XY ||
      y > IST8310_MAX_RAW_XY || y < -IST8310_MAX_RAW_XY ||
      z_sensor > IST8310_MAX_RAW_Z || z_sensor < -IST8310_MAX_RAW_Z) {
    if (++ist_error_count_ >= 5) ist_ok_ = false;
    if (ist_ok_) startIstMeasurement();
    return;
  }
  ist_error_count_ = 0;
  // IST8310's native Z sign is left-handed relative to the X/Y convention.
  // Flip Z so /neo3/mag is a right-handed vector before ROS tilt compensation.
  const int16_t z = static_cast<int16_t>(-z_sensor);
  if (static_cast<uint32_t>(now_ms - last_mag_publish_ms_) >= MAG_PUBLISH_MS) {
    last_mag_publish_ms_ = now_ms;
    publishMag(x, y, z);
  }
  startIstMeasurement();
}

void Neo3Sensors::publishMag(int16_t x, int16_t y, int16_t z) {
  const float mx = static_cast<float>(x) * IST8310_UT_PER_LSB;
  const float my = static_cast<float>(y) * IST8310_UT_PER_LSB;
  const float mz = static_cast<float>(z) * IST8310_UT_PER_LSB;
  const float norm = std::sqrt(mx * mx + my * my + mz * mz);
  ++mag_sequence_;
  char line[128]; CdcLineBuffer out(line, sizeof(line));
  out.print("SENS:MAG:");
  out.print(mag_sequence_); out.print(',');
  out.print(HAL_GetTick()); out.print(',');
  out.print(mx, 2); out.print(',');
  out.print(my, 2); out.print(',');
  out.print(mz, 2); out.print(',');
  out.print(norm, 2); out.print(',');
  out.println(ist_ok_ ? 1 : 0);
  (void)out.flushToUsb();
}

void Neo3Sensors::pollSafetySwitch() {
  const uint32_t now_ms = HAL_GetTick();
  const bool raw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET;
  if (raw != switch_raw_) {
    switch_raw_ = raw;
    switch_changed_ms_ = now_ms;
  }
  if (raw != switch_pressed_ && static_cast<uint32_t>(now_ms - switch_changed_ms_) >= SWITCH_DEBOUNCE_MS) {
    switch_pressed_ = raw;
    char line[64]; CdcLineBuffer out(line, sizeof(line));
    out.print("SENS:SW:");
    out.print(++hw_sequence_); out.print(',');
    out.print(now_ms); out.print(',');
    out.println(switch_pressed_ ? 1 : 0);
    (void)out.flushToUsb();
  }
}

bool Neo3Sensors::gnssAlive(uint32_t now_ms) const {
  // "Alive" means the M9N receiver is actively streaming, independent of fix.
  // This intentionally differs from gnssReady(): indoors NAV-PVT may be fresh
  // at 10 Hz with fix_type=0/sat=0, which is CONNECTED but NOT FIXED.
  if (pvt_.received_ms != 0U && static_cast<uint32_t>(now_ms - pvt_.received_ms) <= PVT_STALE_MS) return true;
  return nmea_.gga_ms != 0U && static_cast<uint32_t>(now_ms - nmea_.gga_ms) <= NMEA_FRESH_MS;
}

bool Neo3Sensors::gnssReady(uint32_t now_ms) const {
  if (pvt_.valid && static_cast<uint32_t>(now_ms - pvt_.received_ms) <= PVT_STALE_MS) {
    const bool type_ok = pvt_.fix_type == 3U || pvt_.fix_type == 4U;
    return pvt_.fix_ok && !pvt_.invalid_llh && type_ok && pvt_.satellites >= 6U &&
      std::isfinite(pvt_.hacc_m) && pvt_.hacc_m > 0.0f && pvt_.hacc_m <= 10.0f;
  }
  return nmea_.gga_valid && static_cast<uint32_t>(now_ms - nmea_.gga_ms) <= NMEA_FRESH_MS &&
    nmea_.satellites >= 6U && nmea_.hdop > 0.0f && nmea_.hdop <= 3.0f;
}

void Neo3Sensors::setSafetyLed(bool on) {
  if (safety_led_on_ == on) return;
  safety_led_on_ = on;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);  // active-low
}

void Neo3Sensors::updateSafetyLed() {
  bool desired = false;
  if (led_mode_ == LedMode::FORCE_ON) desired = true;
  else if (led_mode_ == LedMode::AUTO) desired = gnssReady(HAL_GetTick());
  setSafetyLed(desired);
  last_ready_ = desired;
}

void Neo3Sensors::beep(uint16_t frequency_hz, uint16_t duration_ms) {
  Board_BuzzerStart(frequency_hz, duration_ms);
}

void Neo3Sensors::publishHardwareStatus(bool force) {
  const uint32_t now_ms = HAL_GetTick();
  if (!force && static_cast<uint32_t>(now_ms - last_hw_publish_ms_) < HW_PUBLISH_MS) return;
  last_hw_publish_ms_ = now_ms;
  char line[128]; CdcLineBuffer out(line, sizeof(line));
  out.print("SENS:HW:");
  out.print(++hw_sequence_); out.print(',');
  out.print(now_ms); out.print(',');
  out.print(gnssAlive(now_ms) ? 1 : 0); out.print(',');
  out.print(gnssReady(now_ms) ? 1 : 0); out.print(',');
  out.print(ist_ok_ ? 1 : 0); out.print(',');
  out.print(switch_pressed_ ? 1 : 0); out.print(',');
  out.print(safety_led_on_ ? 1 : 0); out.print(',');
  out.print(config_attempts_); out.print(',');
  out.print(ist_addr_); out.print(',');
  out.print(ist_whoami_); out.print(',');
  out.print(ist_init_error_); out.print(',');
  out.print(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET ? 1 : 0); out.print(',');
  out.println(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET ? 1 : 0);
  (void)out.flushToUsb();
}

int32_t Neo3Sensors::readI32LE(const uint8_t *p) {
  return static_cast<int32_t>(readU32LE(p));
}

uint32_t Neo3Sensors::readU32LE(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t Neo3Sensors::readU16LE(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

float Neo3Sensors::normalize360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}
