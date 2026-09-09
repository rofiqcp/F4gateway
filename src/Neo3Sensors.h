#pragma once

#include "BoardSupport.h"
#include "UsbCdcPort.h"
#include <cstdint>


// CUAV NEO 3 sensor/IO front-end for STM32F411CEU6.
//
// Fixed project wiring:
//   NEO3 TX -> PA3 (USART2 RX)
//   NEO3 RX <- PA2 (USART2 TX)
//   SCL     -> PB8 (I2C1 SCL)
//   SDA     -> PB9 (I2C1 SDA)
//   SAFETY  -> PB12 (active-low input)
//   SW LED  <- PB13 (active-low/open-drain output)
//   BUZZER  <- PA8 (passive buzzer PWM)
//
// The native USB CDC endpoint remains exclusively owned by the HMI/ROS bridge. This
// module multiplexes sensor records onto that same newline-delimited link using
// the SENS:* namespace, so the host never opens a second serial endpoint.
class Neo3Sensors {
public:
  void begin();
  void poll();
  void pollSafetyIo();
  bool safetyPressed() const { return switch_pressed_; }
  bool handleHostCommand(const char *command);

private:
  struct PVT {
    bool valid{false};
    bool fix_ok{false};
    bool invalid_llh{false};
    uint8_t fix_type{0};
    uint8_t satellites{0};
    uint8_t flags2{0};
    uint16_t flags3{0};
    uint32_t itow_ms{0};
    double latitude{0.0};
    double longitude{0.0};
    float altitude_m{0.0f};
    float hacc_m{999.0f};
    float vacc_m{999.0f};
    float vel_n_mps{0.0f};
    float vel_e_mps{0.0f};
    float vel_d_mps{0.0f};
    float ground_speed_mps{0.0f};
    float course_deg_ned{0.0f};
    float sacc_mps{99.0f};
    float course_acc_deg{180.0f};
    float pdop{99.9f};
    float rate_hz{0.0f};
    uint32_t received_ms{0};
  };

  struct NmeaFallback {
    bool gga_valid{false};
    bool rmc_valid{false};
    uint8_t fix_quality{0};
    uint8_t satellites{0};
    double latitude{0.0};
    double longitude{0.0};
    float altitude_m{0.0f};
    float hdop{99.9f};
    float speed_mps{0.0f};
    float course_deg_ned{0.0f};
    uint32_t gga_ms{0};
    uint32_t rmc_ms{0};
  };

  enum class UbxState : uint8_t {
    SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CKA, CKB
  };

  static constexpr uint8_t IST8310_ADDR = 0x0E;
  static constexpr uint8_t IST8310_ADDR_MIN = 0x0C;
  static constexpr uint8_t IST8310_ADDR_MAX = 0x0F;
  static constexpr uint8_t IST8310_WHOAMI_REG = 0x00;
  static constexpr uint8_t IST8310_WHOAMI = 0x10;
  static constexpr uint8_t IST8310J_WHOAMI = 0xA3;
  static constexpr uint8_t IST8310_STAT1 = 0x02;
  static constexpr uint8_t IST8310_DATA_XL = 0x03;
  static constexpr uint8_t IST8310_CTRL1 = 0x0A;
  static constexpr uint8_t IST8310_CTRL2 = 0x0B;
  static constexpr uint8_t IST8310_CTRL3 = 0x0D;
  static constexpr uint8_t IST8310_AVGCNTL = 0x41;
  static constexpr uint8_t IST8310_PDCNTL = 0x42;

  static constexpr uint32_t GNSS_BAUD = 38400;
  static constexpr uint32_t PVT_STALE_MS = 1500;
  static constexpr uint32_t GNSS_CONFIG_RETRY_MS = 3000;
  static constexpr uint32_t GNSS_UART_RETRY_MS = 1000;
  static constexpr uint32_t NMEA_FRESH_MS = 1800;
  static constexpr uint32_t MAG_PUBLISH_MS = 50;     // 20 Hz USB telemetry
  static constexpr uint32_t HW_PUBLISH_MS = 1000;
  static constexpr uint32_t SWITCH_DEBOUNCE_MS = 30;
  static constexpr uint16_t UBX_MAX_PAYLOAD = 128;

  void pollGnss();
  void consumeGnssByte(uint8_t b);
  void consumeUbxByte(uint8_t b);
  void consumeNmeaByte(char c);
  void resetUbx();
  void handleUbxFrame();
  void parseNavPvt();
  void publishPvt();
  void publishNmeaFallback();
  void parseNmeaSentence(char *sentence);
  bool validateNmeaChecksum(const char *sentence) const;
  static bool parseNmeaCoordinate(const char *text, char hemisphere, bool latitude, double &out);
  static uint8_t splitCsv(char *text, char **fields, uint8_t max_fields);

  bool configureGnss();
  bool sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length);
  static void appendU32(uint8_t *payload, uint16_t &pos, uint32_t value);
  static void appendU16(uint8_t *payload, uint16_t &pos, uint16_t value);
  static void appendU8(uint8_t *payload, uint16_t &pos, uint8_t value);

  bool initIst8310();
  void recoverIstBus();
  void pollIst8310();
  bool istRead(uint8_t reg, uint8_t *dst, uint8_t count);
  bool istWrite(uint8_t reg, uint8_t value);
  bool istReadAt(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t count);
  bool istWriteAt(uint8_t addr, uint8_t reg, uint8_t value);
  void startIstMeasurement();
  void publishMag(int16_t x, int16_t y, int16_t z);

  void pollSafetySwitch();
  void updateSafetyLed();
  void publishHardwareStatus(bool force = false);
  bool gnssAlive(uint32_t now_ms) const;
  bool gnssReady(uint32_t now_ms) const;
  void setSafetyLed(bool on);
  void beep(uint16_t frequency_hz, uint16_t duration_ms);

  static int32_t readI32LE(const uint8_t *p);
  static uint32_t readU32LE(const uint8_t *p);
  static uint16_t readU16LE(const uint8_t *p);
  static float normalize360(float deg);


  UbxState ubx_state_{UbxState::SYNC1};
  uint8_t ubx_class_{0};
  uint8_t ubx_id_{0};
  uint16_t ubx_length_{0};
  uint16_t ubx_pos_{0};
  uint8_t ubx_ck_a_{0};
  uint8_t ubx_ck_b_{0};
  uint8_t ubx_rx_ck_a_{0};
  uint8_t ubx_payload_[UBX_MAX_PAYLOAD]{};

  char nmea_line_[128]{};
  uint8_t nmea_len_{0};
  bool nmea_collecting_{false};

  PVT pvt_{};
  NmeaFallback nmea_{};
  bool have_last_pvt_itow_{false};
  uint32_t last_pvt_itow_{0};
  uint32_t last_config_ms_{0};
  uint8_t config_attempts_{0};
  bool gnss_uart_ok_{false};
  uint32_t last_gnss_uart_retry_ms_{0};
  uint32_t gnss_uart_error_count_{0};
  uint32_t gnss_sequence_{0};
  uint32_t mag_sequence_{0};
  uint32_t hw_sequence_{0};

  bool ist_ok_{false};
  uint8_t ist_addr_{IST8310_ADDR};
  uint8_t ist_whoami_{0};
  uint8_t ist_init_error_{0};
  uint32_t last_ist_retry_ms_{0};
  uint32_t last_ist_measurement_ms_{0};
  uint32_t last_mag_publish_ms_{0};
  uint8_t ist_error_count_{0};

  bool switch_raw_{false};
  bool switch_pressed_{false};
  uint32_t switch_changed_ms_{0};
  bool safety_led_on_{false};
  enum class LedMode : uint8_t { AUTO, FORCE_OFF, FORCE_ON };
  LedMode led_mode_{LedMode::AUTO};
  bool last_ready_{false};
  uint32_t last_hw_publish_ms_{0};
};
