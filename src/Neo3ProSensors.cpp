#ifdef NEO3PRO

#include "Neo3ProSensors.h"
#include "BoardSupport.h"
#include "UsbCdcPort.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr uint16_t DTID_NODE_STATUS = 341U;
constexpr uint64_t SIG_NODE_STATUS = 0x0F0868D0C1A7C6F1ULL;
constexpr uint16_t DTID_MAG1 = 1001U;
constexpr uint64_t SIG_MAG1 = 0xE2A7D4A9460BC2F2ULL;
constexpr uint16_t DTID_MAG2 = 1002U;
constexpr uint64_t SIG_MAG2 = 0xB6AC0C442430297EULL;
constexpr uint16_t DTID_PRESSURE = 1028U;
constexpr uint64_t SIG_PRESSURE = 0xCDC7C43412BDC89AULL;
constexpr uint16_t DTID_TEMPERATURE = 1029U;
constexpr uint64_t SIG_TEMPERATURE = 0x49272A6477D96271ULL;
constexpr uint16_t DTID_AUX = 1061U;
constexpr uint64_t SIG_AUX = 0x9BE8BDC4C3DBBFD2ULL;
constexpr uint16_t DTID_FIX2 = 1063U;
constexpr uint64_t SIG_FIX2 = 0xCA41E7000F37435FULL;
constexpr uint16_t DTID_BUTTON = 20001U;
constexpr uint64_t SIG_BUTTON = 0x0645A46EFBA7466EULL;
constexpr uint16_t DTID_GNSS_HEADING = 20002U;
constexpr uint64_t SIG_GNSS_HEADING = 0x315CAE39ECED3412ULL;
constexpr uint16_t DTID_GNSS_STATUS = 20003U;
constexpr uint64_t SIG_GNSS_STATUS = 0xBA3CB4ABBB007F69ULL;

constexpr uint8_t MCP_RESET = 0xC0U;
constexpr uint8_t MCP_READ = 0x03U;
constexpr uint8_t MCP_WRITE = 0x02U;
constexpr uint8_t MCP_BITMOD = 0x05U;
constexpr uint8_t MCP_READ_RX0 = 0x90U;
constexpr uint8_t MCP_READ_RX1 = 0x94U;
constexpr uint8_t REG_CANSTAT = 0x0EU;
constexpr uint8_t REG_CANCTRL = 0x0FU;
constexpr uint8_t REG_TEC = 0x1CU;
constexpr uint8_t REG_REC = 0x1DU;
constexpr uint8_t REG_CNF3 = 0x28U;
constexpr uint8_t REG_CNF2 = 0x29U;
constexpr uint8_t REG_CNF1 = 0x2AU;
constexpr uint8_t REG_CANINTE = 0x2BU;
constexpr uint8_t REG_CANINTF = 0x2CU;
constexpr uint8_t REG_EFLG = 0x2DU;
constexpr uint8_t REG_RXB0CTRL = 0x60U;
constexpr uint8_t REG_RXB1CTRL = 0x70U;
constexpr uint8_t MODE_MASK = 0xE0U;
constexpr uint8_t MODE_CONFIG = 0x80U;
constexpr uint8_t MODE_NORMAL = 0x00U;
constexpr uint8_t RX0IF = 0x01U;
constexpr uint8_t RX1IF = 0x02U;
constexpr uint8_t ERRIF = 0x20U;
constexpr uint8_t RX0OVR = 0x40U;
constexpr uint8_t RX1OVR = 0x80U;
constexpr uint32_t MCP_SPI_PRESCALER = SPI_BAUDRATEPRESCALER_16; // 96/16=6 MHz, MCP2515 max 10 MHz
constexpr uint32_t PROFILE_PROBE_MS = 1400U;
constexpr uint32_t HW_PUBLISH_MS = 1000U;
constexpr uint32_t GNSS_STALE_MS = 1500U;
constexpr uint32_t MAG_STALE_MS = 800U;
constexpr uint32_t BUTTON_STALE_MS = 250U; // AP_Periph transmits safety Button at 10 Hz while held
constexpr uint8_t MAX_RX_PER_POLL = 24U;
constexpr uint8_t MAX_RX_REALTIME = 4U;
constexpr uint64_t GPS_WEEK_USEC = 604800000000ULL;

uint16_t crc16Ccitt(const char *s) {
  uint16_t crc = 0xFFFFU;
  if (!s) return crc;
  while (*s) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*s++)) << 8U;
    for (uint8_t i = 0U; i < 8U; ++i)
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1U);
  }
  return crc;
}

bool decodeScalar(const CanardRxTransfer *t, uint32_t ofs, uint8_t bits,
                  bool sign, void *out) {
  return t && out && canardDecodeScalar(t, ofs, bits, sign, out) == bits;
}

float decodeF16(const CanardRxTransfer *t, uint32_t ofs, bool *ok = nullptr) {
  uint16_t raw = 0U;
  const bool good = decodeScalar(t, ofs, 16U, false, &raw);
  if (ok) *ok = good;
  return good ? canardConvertFloat16ToNativeFloat(raw) : NAN;
}

float finiteOr(float value, float fallback) {
  return std::isfinite(value) ? value : fallback;
}
} // namespace

bool Neo3ProSensors::begin() {
  canardInit(&canard_, canard_pool_, sizeof(canard_pool_), &Neo3ProSensors::onTransfer,
             &Neo3ProSensors::shouldAccept, this);
  probe_index_ = 0U;
  return initMcp(8U);
}

bool Neo3ProSensors::mcpTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (!tx || len == 0U) return false;
  if (!Board_SpiAcquire(BoardSpiOwner::NEO3PRO_MCP2515, MCP_SPI_PRESCALER)) {
    ++spi_errors_;
    return false;
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_StatusTypeDef st = rx
      ? HAL_SPI_TransmitReceive(&hspi1, const_cast<uint8_t *>(tx), rx, len, 4U)
      : HAL_SPI_Transmit(&hspi1, const_cast<uint8_t *>(tx), len, 4U);
  const uint32_t busy_start = DWT->CYCCNT;
  const uint32_t busy_limit = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U) * 2500U;
  while ((SPI1->SR & SPI_SR_BSY) != 0U &&
         static_cast<uint32_t>(DWT->CYCCNT - busy_start) < busy_limit) {}
  if ((SPI1->SR & SPI_SR_BSY) != 0U) st = HAL_TIMEOUT;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  Board_SpiRelease(BoardSpiOwner::NEO3PRO_MCP2515);
  if (st != HAL_OK) ++spi_errors_;
  return st == HAL_OK;
}

bool Neo3ProSensors::mcpWrite(uint8_t addr, uint8_t value) {
  const uint8_t tx[3] = {MCP_WRITE, addr, value};
  return mcpTransfer(tx, nullptr, sizeof(tx));
}

bool Neo3ProSensors::mcpRead(uint8_t addr, uint8_t *value) {
  if (!value) return false;
  const uint8_t tx[3] = {MCP_READ, addr, 0xFFU};
  uint8_t rx[3]{};
  if (!mcpTransfer(tx, rx, sizeof(tx))) return false;
  *value = rx[2];
  return true;
}

bool Neo3ProSensors::mcpBitModify(uint8_t addr, uint8_t mask, uint8_t value) {
  const uint8_t tx[4] = {MCP_BITMOD, addr, mask, value};
  return mcpTransfer(tx, nullptr, sizeof(tx));
}

bool Neo3ProSensors::mcpSetMode(uint8_t mode) {
  if (!mcpBitModify(REG_CANCTRL, MODE_MASK, mode)) return false;
  const uint32_t started = HAL_GetTick();
  do {
    uint8_t stat = 0U;
    if (!mcpRead(REG_CANSTAT, &stat)) return false;
    if ((stat & MODE_MASK) == mode) return true;
  } while (static_cast<uint32_t>(HAL_GetTick() - started) < 20U);
  return false;
}

bool Neo3ProSensors::initMcp(uint8_t osc_mhz) {
  Board_SpiDeselectAll();
  const uint8_t reset = MCP_RESET;
  if (!mcpTransfer(&reset, nullptr, 1U)) { can_ok_ = false; return false; }
  Board_RealtimeDelayMs(3U);
  if (!mcpSetMode(MODE_CONFIG)) { can_ok_ = false; return false; }

  // DroneCAN classic CAN runs at 1 Mbit/s. Both profiles use 75% sample point:
  // 8 MHz: 4 TQ/bit, 16 MHz: 8 TQ/bit. Auto-probe locks only after a valid
  // complete DroneCAN transfer, not merely after electrical noise/raw frames.
  uint8_t cnf1 = 0x00U, cnf2 = 0xC0U, cnf3 = 0x80U; // 8 MHz crystal
  if (osc_mhz == 16U) { cnf1 = 0x00U; cnf2 = 0xCAU; cnf3 = 0x81U; }
  if (!mcpWrite(REG_CNF1, cnf1) || !mcpWrite(REG_CNF2, cnf2) ||
      !mcpWrite(REG_CNF3, cnf3)) { can_ok_ = false; return false; }

  // Receive all valid frames; DroneCAN itself filters by transfer type/DTID.
  // RXB0 rollover protects against short HMI SPI bursts.
  if (!mcpWrite(REG_RXB0CTRL, 0x64U) || !mcpWrite(REG_RXB1CTRL, 0x60U) ||
      !mcpWrite(REG_CANINTE, RX0IF | RX1IF | ERRIF) ||
      !mcpWrite(REG_CANINTF, 0x00U) || !mcpWrite(REG_EFLG, 0x00U) ||
      !mcpSetMode(MODE_NORMAL)) { can_ok_ = false; return false; }

  active_osc_mhz_ = osc_mhz;
  profile_started_ms_ = HAL_GetTick();
  last_raw_frame_ms_ = 0U;
  can_ok_ = true;
  return true;
}

void Neo3ProSensors::maybeProbeOscillator(uint32_t now_ms) {
  if (oscillator_locked_) return;
  if (static_cast<uint32_t>(now_ms - profile_started_ms_) < PROFILE_PROBE_MS) return;
  probe_index_ ^= 1U;
  ++recoveries_;
  (void)initMcp(probe_index_ == 0U ? 8U : 16U);
}

bool Neo3ProSensors::readOneCanFrame(uint8_t buffer_index) {
  uint8_t tx[14]{};
  uint8_t rx[14]{};
  tx[0] = buffer_index == 0U ? MCP_READ_RX0 : MCP_READ_RX1;
  for (uint8_t i = 1U; i < sizeof(tx); ++i) tx[i] = 0xFFU;
  if (!mcpTransfer(tx, rx, sizeof(tx))) return false;
  const uint8_t sidh = rx[1], sidl = rx[2], eid8 = rx[3], eid0 = rx[4];
  const uint8_t dlc = static_cast<uint8_t>(rx[5] & 0x0FU);
  if (dlc > 8U) { ++can_decode_errors_; return false; }
  const bool extended = (sidl & 0x08U) != 0U;
  uint32_t id = 0U;
  if (extended) {
    id = (static_cast<uint32_t>(sidh) << 21U) |
         (static_cast<uint32_t>(sidl & 0xE0U) << 13U) |
         (static_cast<uint32_t>(sidl & 0x03U) << 16U) |
         (static_cast<uint32_t>(eid8) << 8U) | eid0;
  } else {
    id = (static_cast<uint32_t>(sidh) << 3U) | (sidl >> 5U);
  }
  ++raw_frames_;
  last_raw_frame_ms_ = HAL_GetTick();
  if (!extended) return true; // DroneCAN v0 uses CAN 2.0B 29-bit identifiers.

  CanardCANFrame frame{};
  frame.id = id | CANARD_CAN_FRAME_EFF;
  frame.data_len = dlc;
  frame.iface_id = 0U;
  std::memcpy(frame.data, &rx[6], dlc);
  const int16_t res = canardHandleRxFrame(&canard_, &frame,
      static_cast<uint64_t>(HAL_GetTick()) * 1000ULL);
  if (res < 0) ++can_decode_errors_;
  return true;
}

bool Neo3ProSensors::drainCan(uint8_t frame_budget, bool allow_recovery) {
  while (frame_budget-- > 0U) {
    uint8_t flags = 0U;
    if (!mcpRead(REG_CANINTF, &flags)) {
      if (allow_recovery) recoverCan();
      return false;
    }
    if ((flags & (RX0IF | RX1IF)) == 0U) {
      if (allow_recovery && (flags & ERRIF)) {
        uint8_t eflg = 0U;
        if (mcpRead(REG_EFLG, &eflg) && (eflg & (RX0OVR | RX1OVR))) {
          ++can_overflows_;
          (void)mcpBitModify(REG_EFLG, RX0OVR | RX1OVR, 0U);
        }
        (void)mcpBitModify(REG_CANINTF, ERRIF, 0U);
      }
      return true;
    }
    if (flags & RX0IF) {
      if (!readOneCanFrame(0U) && !allow_recovery) return false;
      (void)mcpBitModify(REG_CANINTF, RX0IF, 0U);
    }
    if ((flags & RX1IF) && frame_budget > 0U) {
      --frame_budget;
      if (!readOneCanFrame(1U) && !allow_recovery) return false;
      (void)mcpBitModify(REG_CANINTF, RX1IF, 0U);
    }
  }
  return true;
}

void Neo3ProSensors::recoverCan() {
  ++recoveries_;
  oscillator_locked_ = false;
  can_ok_ = initMcp(active_osc_mhz_ == 16U ? 16U : 8U);
}

void Neo3ProSensors::pollRealtime() {
  if (!can_ok_ || Board_SpiOwner() != BoardSpiOwner::NONE) return;
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET) return;
  // Bounded drain only. Never reset/probe the shared bus from a TFT yield.
  (void)drainCan(MAX_RX_REALTIME, false);
}

void Neo3ProSensors::pollSafetyIo() {
  // Momentary DroneCAN safety button state is freshness-based; no local GPIO.
}

bool Neo3ProSensors::safetyPressed() const {
  return button_.seen && button_.button == 1U &&
         static_cast<uint32_t>(HAL_GetTick() - button_.received_ms) <= BUTTON_STALE_MS;
}

void Neo3ProSensors::poll() {
  const uint32_t now_ms = HAL_GetTick();
  if (!can_ok_) {
    if (static_cast<uint32_t>(now_ms - last_poll_ms_) >= 500U) {
      last_poll_ms_ = now_ms;
      recoverCan();
    }
    publishHardware(false);
    return;
  }
  if (Board_SpiOwner() == BoardSpiOwner::NONE) {
    const bool int_active = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET;
    if (int_active || static_cast<uint32_t>(now_ms - last_poll_ms_) >= 10U) {
      last_poll_ms_ = now_ms;
      (void)drainCan(MAX_RX_PER_POLL, true);
    }
  }
  if (static_cast<uint32_t>(now_ms - last_cleanup_ms_) >= 1000U) {
    last_cleanup_ms_ = now_ms;
    canardCleanupStaleTransfers(&canard_, static_cast<uint64_t>(now_ms) * 1000ULL);
  }
  if (button_.published_pressed && !safetyPressed()) {
    button_.published_pressed = false;
    publishButton(false);
  }
  maybeProbeOscillator(now_ms);
  publishHardware(false);
}

bool Neo3ProSensors::acceptSource(uint8_t source_node_id) const {
  return primary_node_id_ == 0U || source_node_id == primary_node_id_;
}

bool Neo3ProSensors::shouldAccept(const CanardInstance *, uint64_t *signature,
                                  uint16_t id, CanardTransferType type, uint8_t) {
  if (!signature || type != CanardTransferTypeBroadcast) return false;
  switch (id) {
    case DTID_NODE_STATUS: *signature = SIG_NODE_STATUS; return true;
    case DTID_MAG1: *signature = SIG_MAG1; return true;
    case DTID_MAG2: *signature = SIG_MAG2; return true;
    case DTID_PRESSURE: *signature = SIG_PRESSURE; return true;
    case DTID_TEMPERATURE: *signature = SIG_TEMPERATURE; return true;
    case DTID_AUX: *signature = SIG_AUX; return true;
    case DTID_FIX2: *signature = SIG_FIX2; return true;
    case DTID_BUTTON: *signature = SIG_BUTTON; return true;
    case DTID_GNSS_HEADING: *signature = SIG_GNSS_HEADING; return true;
    case DTID_GNSS_STATUS: *signature = SIG_GNSS_STATUS; return true;
    default: return false;
  }
}

void Neo3ProSensors::onTransfer(CanardInstance *ins, CanardRxTransfer *transfer) {
  if (!ins || !transfer) return;
  auto *self = static_cast<Neo3ProSensors *>(canardGetUserReference(ins));
  if (!self) return;
  ++self->dronecan_transfers_;
  self->oscillator_locked_ = true; // only a CRC-valid/reassembled transfer reaches here

  if (transfer->data_type_id == DTID_FIX2 && self->primary_node_id_ == 0U)
    self->primary_node_id_ = transfer->source_node_id;
  if (!self->acceptSource(transfer->source_node_id)) {
    ++self->foreign_node_drops_;
    return;
  }
  ++self->accepted_transfers_;
  switch (transfer->data_type_id) {
    case DTID_FIX2: self->decodeFix2(transfer); break;
    case DTID_AUX: self->decodeAuxiliary(transfer); break;
    case DTID_MAG1: self->decodeMag(transfer, false); break;
    case DTID_MAG2: self->decodeMag(transfer, true); break;
    case DTID_PRESSURE: self->decodeStaticPressure(transfer); break;
    case DTID_TEMPERATURE: self->decodeStaticTemperature(transfer); break;
    case DTID_NODE_STATUS: self->decodeNodeStatus(transfer); break;
    case DTID_GNSS_STATUS: self->decodeGnssStatus(transfer); break;
    case DTID_GNSS_HEADING: self->decodeHeading(transfer); break;
    case DTID_BUTTON: self->decodeButton(transfer); break;
    default: break;
  }
}

void Neo3ProSensors::decodeFix2(const CanardRxTransfer *t) {
  uint64_t timestamp = 0U, gnss_timestamp = 0U;
  uint8_t time_standard = 0U, leap = 0U;
  int64_t lon = 0, lat = 0;
  int32_t h_ell = 0, h_msl = 0;
  float vn = 0.0F, ve = 0.0F, vd = 0.0F;
  uint8_t sats = 0U, status = 0U, mode = 0U, sub = 0U, cov_len = 0U;
  if (!decodeScalar(t, 0U, 56U, false, &timestamp) ||
      !decodeScalar(t, 56U, 56U, false, &gnss_timestamp) ||
      !decodeScalar(t, 112U, 3U, false, &time_standard) ||
      !decodeScalar(t, 128U, 8U, false, &leap) ||
      !decodeScalar(t, 136U, 37U, true, &lon) ||
      !decodeScalar(t, 173U, 37U, true, &lat) ||
      !decodeScalar(t, 210U, 27U, true, &h_ell) ||
      !decodeScalar(t, 237U, 27U, true, &h_msl) ||
      !decodeScalar(t, 264U, 32U, true, &vn) ||
      !decodeScalar(t, 296U, 32U, true, &ve) ||
      !decodeScalar(t, 328U, 32U, true, &vd) ||
      !decodeScalar(t, 360U, 6U, false, &sats) ||
      !decodeScalar(t, 366U, 2U, false, &status) ||
      !decodeScalar(t, 368U, 4U, false, &mode) ||
      !decodeScalar(t, 372U, 6U, false, &sub) ||
      !decodeScalar(t, 378U, 6U, false, &cov_len) || cov_len > 36U) {
    ++can_decode_errors_;
    return;
  }

  uint32_t ofs = 384U;
  float cov[36]{};
  for (uint8_t i = 0U; i < cov_len; ++i, ofs += 16U) {
    bool ok = false;
    cov[i] = decodeF16(t, ofs, &ok);
    if (!ok) { ++can_decode_errors_; return; }
  }
  bool pdop_ok = false;
  const float pdop = decodeF16(t, ofs, &pdop_ok);
  if (!pdop_ok) { ++can_decode_errors_; return; }
  ofs += 16U;

  const uint32_t now = HAL_GetTick();
  const uint32_t delta = gnss_.seen ? static_cast<uint32_t>(now - gnss_.received_ms) : 0U;
  gnss_.seen = true;
  gnss_.source_node_id = t->source_node_id;
  gnss_.last_interval_ms = delta;
  if (delta > 0U && delta <= 2000U) {
    const float hz = 1000.0F / static_cast<float>(delta);
    gnss_.rate_hz = gnss_.rate_hz > 0.0F ? 0.85F * gnss_.rate_hz + 0.15F * hz : hz;
  }
  gnss_.received_ms = now;
  gnss_.timestamp_usec = timestamp;
  gnss_.gnss_timestamp_usec = gnss_timestamp;
  gnss_.gnss_time_standard = time_standard;
  gnss_.num_leap_seconds = leap;
  gnss_.lon_e8 = lon;
  gnss_.lat_e8 = lat;
  gnss_.height_ellipsoid_mm = h_ell;
  gnss_.height_msl_mm = h_msl;
  gnss_.vel_n = vn; gnss_.vel_e = ve; gnss_.vel_d = vd;
  gnss_.sats = sats;
  gnss_.status = status;
  gnss_.mode = mode;
  gnss_.sub_mode = sub;
  gnss_.covariance_len = cov_len;
  std::fill_n(gnss_.covariance, 36U, NAN);
  for (uint8_t i = 0U; i < cov_len; ++i) gnss_.covariance[i] = cov[i];
  gnss_.pdop = std::isfinite(pdop) ? pdop : gnss_.pdop;

  // ArduPilot/AP_Periph canonical Fix2 covariance contract is len=6:
  // [hacc^2,hacc^2,vacc^2,sacc^2,sacc^2,sacc^2]. Do NOT treat it as a 6x6 matrix.
  gnss_.pos_cov_valid = cov_len == 6U && std::isfinite(cov[0]) &&
      std::isfinite(cov[1]) && std::isfinite(cov[2]) &&
      cov[0] >= 0.0F && cov[1] >= 0.0F && cov[2] >= 0.0F;
  gnss_.vel_cov_valid = cov_len == 6U && std::isfinite(cov[3]) &&
      std::isfinite(cov[4]) && std::isfinite(cov[5]) &&
      cov[3] >= 0.0F && cov[4] >= 0.0F && cov[5] >= 0.0F;
  if (gnss_.pos_cov_valid) {
    gnss_.hacc_m = std::sqrt(cov[0]);
    gnss_.vacc_m = std::sqrt(cov[2]);
  }
  if (gnss_.vel_cov_valid)
    gnss_.sacc_mps = std::sqrt((cov[3] + cov[4] + cov[5]) / 3.0F);

  ecef_.valid = false;
  if (static_cast<uint32_t>(t->payload_len) * 8U >= ofs + 216U)
    decodeEcef(t, ofs);

  // Publish metadata/covariance first so the ROS bridge can atomically enrich
  // the immediately following compatibility SENS:GNSS sample.
  publishGnssPro();
  publishGnssCov();
  if (ecef_.valid) publishEcef();
  publishGnss();
}

void Neo3ProSensors::decodeEcef(const CanardRxTransfer *t, uint32_t ofs) {
  EcefState next{};
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  for (uint8_t i = 0U; i < 3U; ++i) {
    if (!decodeScalar(t, ofs + static_cast<uint32_t>(i) * 32U, 32U, true,
                      &next.velocity_xyz[i])) { ++can_decode_errors_; return; }
  }
  ofs += 96U;
  for (uint8_t i = 0U; i < 3U; ++i) {
    if (!decodeScalar(t, ofs + static_cast<uint32_t>(i) * 36U, 36U, true,
                      &next.position_xyz_mm[i])) { ++can_decode_errors_; return; }
  }
  ofs += 108U + 6U; // void6 alignment
  if (!decodeScalar(t, ofs, 6U, false, &next.covariance_len) || next.covariance_len > 36U) {
    ++can_decode_errors_; return;
  }
  ofs += 6U;
  for (uint8_t i = 0U; i < next.covariance_len; ++i, ofs += 16U) {
    bool ok = false;
    next.covariance[i] = decodeF16(t, ofs, &ok);
    if (!ok) { ++can_decode_errors_; return; }
  }
  next.valid = true;
  ecef_ = next;
}

void Neo3ProSensors::decodeAuxiliary(const CanardRxTransfer *t) {
  bool ok = false;
  const float gdop = decodeF16(t, 0U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float pdop = decodeF16(t, 16U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float hdop = decodeF16(t, 32U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float vdop = decodeF16(t, 48U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float tdop = decodeF16(t, 64U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float ndop = decodeF16(t, 80U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float edop = decodeF16(t, 96U, &ok); if (!ok) { ++can_decode_errors_; return; }
  uint8_t visible = 0U, used = 0U;
  if (!decodeScalar(t, 112U, 7U, false, &visible) ||
      !decodeScalar(t, 119U, 6U, false, &used)) { ++can_decode_errors_; return; }
  gnss_.gdop = gdop; gnss_.pdop = pdop; gnss_.hdop = hdop; gnss_.vdop = vdop;
  gnss_.tdop = tdop; gnss_.ndop = ndop; gnss_.edop = edop;
  gnss_.sats_visible = visible;
  if (used > 0U) gnss_.sats = used;
  // Auxiliary may follow Fix2; publish metadata immediately without duplicating
  // the position sample.
  if (gnss_.seen) publishGnssPro();
}

void Neo3ProSensors::decodeMag(const CanardRxTransfer *t, bool v2) {
  uint32_t ofs = 0U;
  uint8_t sid = 0U;
  if (v2) {
    if (!decodeScalar(t, 0U, 8U, false, &sid)) { ++can_decode_errors_; return; }
    ofs = 8U;
  }
  bool ok = false;
  const float x = decodeF16(t, ofs, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float y = decodeF16(t, ofs + 16U, &ok); if (!ok) { ++can_decode_errors_; return; }
  const float z = decodeF16(t, ofs + 32U, &ok); if (!ok) { ++can_decode_errors_; return; }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    ++can_decode_errors_; return;
  }
  ofs += 48U;
  const uint32_t total_bits = static_cast<uint32_t>(t->payload_len) * 8U;
  uint8_t cov_len = 0U;
  if (total_bits > ofs) {
    const uint32_t remaining = total_bits - ofs;
    // Tail-array optimization: covariance has no explicit length on classic CAN.
    cov_len = static_cast<uint8_t>(std::min<uint32_t>(9U, remaining / 16U));
  }
  mag_.seen = true;
  mag_.source_node_id = t->source_node_id;
  mag_.received_ms = HAL_GetTick();
  mag_.sensor_id = sid;
  mag_.message_type = v2 ? 2U : 1U;
  mag_.x_ga = x; mag_.y_ga = y; mag_.z_ga = z;
  mag_.covariance_len = cov_len;
  std::fill_n(mag_.covariance, 9U, NAN);
  for (uint8_t i = 0U; i < cov_len; ++i) {
    mag_.covariance[i] = decodeF16(t, ofs + static_cast<uint32_t>(i) * 16U, &ok);
    if (!ok) { ++can_decode_errors_; mag_.covariance_len = 0U; break; }
  }
  publishMagPro();
  publishMag();
}

void Neo3ProSensors::decodeStaticPressure(const CanardRxTransfer *t) {
  float pressure = NAN;
  bool ok = false;
  if (!decodeScalar(t, 0U, 32U, true, &pressure)) { ++can_decode_errors_; return; }
  const float variance = decodeF16(t, 32U, &ok);
  if (!ok || !std::isfinite(pressure) || pressure <= 0.0F) { ++can_decode_errors_; return; }
  baro_.pressure_seen = true;
  baro_.source_node_id = t->source_node_id;
  baro_.pressure_ms = HAL_GetTick();
  baro_.pressure_pa = pressure;
  baro_.pressure_variance = variance;
  publishBaroPressure();
}

void Neo3ProSensors::decodeStaticTemperature(const CanardRxTransfer *t) {
  bool ok = false;
  const float temperature = decodeF16(t, 0U, &ok);
  if (!ok) { ++can_decode_errors_; return; }
  const float variance = decodeF16(t, 16U, &ok);
  if (!ok || !std::isfinite(temperature) || temperature <= 0.0F) { ++can_decode_errors_; return; }
  baro_.temperature_seen = true;
  baro_.source_node_id = t->source_node_id;
  baro_.temperature_ms = HAL_GetTick();
  baro_.temperature_k = temperature;
  baro_.temperature_variance = variance;
  publishBaroTemperature();
}

void Neo3ProSensors::decodeNodeStatus(const CanardRxTransfer *t) {
  NodeState next{};
  next.seen = true;
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  if (!decodeScalar(t, 0U, 32U, false, &next.uptime_sec) ||
      !decodeScalar(t, 32U, 2U, false, &next.health) ||
      !decodeScalar(t, 34U, 3U, false, &next.mode) ||
      !decodeScalar(t, 37U, 3U, false, &next.sub_mode) ||
      !decodeScalar(t, 40U, 16U, false, &next.vendor_status)) {
    ++can_decode_errors_; return;
  }
  node_ = next;
  publishNodeStatus();
}

void Neo3ProSensors::decodeGnssStatus(const CanardRxTransfer *t) {
  GnssStatusState next{};
  next.seen = true;
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  uint8_t healthy = 0U;
  if (!decodeScalar(t, 0U, 32U, false, &next.error_codes) ||
      !decodeScalar(t, 32U, 1U, false, &healthy) ||
      !decodeScalar(t, 33U, 23U, false, &next.status_bits)) {
    ++can_decode_errors_; return;
  }
  next.healthy = healthy != 0U;
  gnss_status_ = next;
  publishGnssStatus();
}

void Neo3ProSensors::decodeHeading(const CanardRxTransfer *t) {
  HeadingState next{};
  next.seen = true;
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  uint8_t valid = 0U, acc_valid = 0U;
  if (!decodeScalar(t, 0U, 1U, false, &valid) ||
      !decodeScalar(t, 1U, 1U, false, &acc_valid)) { ++can_decode_errors_; return; }
  bool ok = false;
  next.heading_rad = decodeF16(t, 2U, &ok); if (!ok) { ++can_decode_errors_; return; }
  next.accuracy_rad = decodeF16(t, 18U, &ok); if (!ok) { ++can_decode_errors_; return; }
  next.valid = valid != 0U && std::isfinite(next.heading_rad);
  next.accuracy_valid = acc_valid != 0U && std::isfinite(next.accuracy_rad);
  heading_ = next;
  publishHeading();
}

void Neo3ProSensors::decodeButton(const CanardRxTransfer *t) {
  ButtonState next = button_;
  next.seen = true;
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  if (!decodeScalar(t, 0U, 8U, false, &next.button) ||
      !decodeScalar(t, 8U, 8U, false, &next.press_time)) { ++can_decode_errors_; return; }
  button_ = next;
  if (button_.button == 1U) {
    button_.published_pressed = true;
    publishButton(true);
  }
}

void Neo3ProSensors::writeUsbLine(const char *line) {
  if (!line) return;
  const size_t n = strnlen(line, 620U);
  if (n == 0U || n >= 620U || gUsb.availableForWrite() < static_cast<int>(n + 1U)) return;
  (void)gUsb.write(reinterpret_cast<const uint8_t *>(line), n);
  const uint8_t nl = '\n';
  (void)gUsb.write(&nl, 1U);
}

void Neo3ProSensors::writeV2Record(const char *prefix, const char *body) {
  if (!prefix || !body) return;
  char line[640];
  const uint16_t crc = crc16Ccitt(body);
  const int n = std::snprintf(line, sizeof(line), "%s%s,2,%u", prefix, body,
                              static_cast<unsigned>(crc));
  if (n > 0 && static_cast<size_t>(n) < sizeof(line)) writeUsbLine(line);
}

uint32_t Neo3ProSensors::gnssTowMs() const {
  if (gnss_.gnss_timestamp_usec == 0U) return 0U;
  int64_t gps_usec = static_cast<int64_t>(gnss_.gnss_timestamp_usec);
  switch (gnss_.gnss_time_standard) {
    case 1U: // TAI = GPS + 19 s
      gps_usec -= 19000000LL;
      break;
    case 2U: // UTC = GPS - leap_seconds + 9 s
      if (gnss_.num_leap_seconds == 0U) return 0U;
      gps_usec += (static_cast<int64_t>(gnss_.num_leap_seconds) - 9LL) * 1000000LL;
      break;
    case 3U: // GPS
      break;
    default:
      return 0U;
  }
  if (gps_usec < 0) return 0U;
  return static_cast<uint32_t>((static_cast<uint64_t>(gps_usec) % GPS_WEEK_USEC) / 1000ULL);
}

void Neo3ProSensors::publishGnss() {
  if (!gnss_.seen) return;
  const uint32_t seq = ++gnss_sequence_;
  const float gs = std::sqrt(gnss_.vel_n * gnss_.vel_n + gnss_.vel_e * gnss_.vel_e);
  float course = std::atan2(gnss_.vel_e, gnss_.vel_n) * 57.2957795F;
  if (course < 0.0F) course += 360.0F;
  const bool fix_ok = gnss_.status >= 2U;
  const double lat = static_cast<double>(gnss_.lat_e8) * 1.0e-8;
  const double lon = static_cast<double>(gnss_.lon_e8) * 1.0e-8;
  const bool llh_bad = !std::isfinite(lat) || !std::isfinite(lon) ||
      std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0 ||
      (gnss_.lat_e8 == 0 && gnss_.lon_e8 == 0);

  // Exact legacy v2 payload/trailer contract consumed by stmf4:
  // SENS:GNSS:<23 numeric fields>,2,<decimal CRC over the 23 fields>.
  char body[470];
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%lu,%u,%u,%u,%u,%.8f,%.8f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f,%.2f,%.0f,%.0f",
      static_cast<unsigned long>(seq), static_cast<unsigned long>(gnss_.received_ms),
      static_cast<unsigned long>(gnssTowMs()), static_cast<unsigned>(gnss_.status),
      fix_ok ? 1U : 0U, llh_bad ? 1U : 0U, static_cast<unsigned>(gnss_.sats),
      lat, lon, static_cast<double>(gnss_.height_msl_mm) * 1.0e-3,
      finiteOr(gnss_.hacc_m, 100.0F), finiteOr(gnss_.vacc_m, 150.0F),
      gnss_.vel_n, gnss_.vel_e, gnss_.vel_d, gs, course,
      finiteOr(gnss_.sacc_mps, 5.0F),
      heading_.accuracy_valid ? heading_.accuracy_rad * 57.2957795F : 90.0F,
      finiteOr(gnss_.pdop, 99.0F), finiteOr(gnss_.rate_hz, 0.0F),
      static_cast<double>(gnss_.mode), static_cast<double>(gnss_.sub_mode));
  writeV2Record("SENS:GNSS:", body);
}

void Neo3ProSensors::publishGnssPro() {
  if (!gnss_.seen) return;
  char body[600];
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%llu,%llu,%u,%u,%u,%u,%u,%u,%u,%ld,%ld,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.3f,%u,%u",
      static_cast<unsigned long>(++gnss_pro_sequence_),
      static_cast<unsigned long>(gnss_.received_ms), static_cast<unsigned>(gnss_.source_node_id),
      static_cast<unsigned long long>(gnss_.timestamp_usec),
      static_cast<unsigned long long>(gnss_.gnss_timestamp_usec),
      static_cast<unsigned>(gnss_.gnss_time_standard), static_cast<unsigned>(gnss_.num_leap_seconds),
      static_cast<unsigned>(gnss_.status), static_cast<unsigned>(gnss_.mode),
      static_cast<unsigned>(gnss_.sub_mode), static_cast<unsigned>(gnss_.sats),
      static_cast<unsigned>(gnss_.sats_visible), static_cast<long>(gnss_.height_msl_mm),
      static_cast<long>(gnss_.height_ellipsoid_mm), finiteOr(gnss_.gdop, -1.0F),
      finiteOr(gnss_.pdop, -1.0F), finiteOr(gnss_.hdop, -1.0F), finiteOr(gnss_.vdop, -1.0F),
      finiteOr(gnss_.tdop, -1.0F), finiteOr(gnss_.ndop, -1.0F), finiteOr(gnss_.edop, -1.0F),
      finiteOr(gnss_.rate_hz, 0.0F), gnss_.pos_cov_valid ? 1U : 0U, gnss_.vel_cov_valid ? 1U : 0U);
  writeV2Record("SENS:GNSSPRO:", body);
}

void Neo3ProSensors::publishGnssCov() {
  if (!gnss_.seen) return;
  char body[600];
  int n = std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u",
      static_cast<unsigned long>(++gnss_cov_sequence_), static_cast<unsigned long>(gnss_.received_ms),
      static_cast<unsigned>(gnss_.source_node_id), static_cast<unsigned>(gnss_.covariance_len));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return;
  size_t pos = static_cast<size_t>(n);
  for (uint8_t i = 0U; i < gnss_.covariance_len && pos < sizeof(body); ++i) {
    const int m = std::snprintf(body + pos, sizeof(body) - pos, ",%.6g",
                                finiteOr(gnss_.covariance[i], -1.0F));
    if (m <= 0 || static_cast<size_t>(m) >= sizeof(body) - pos) return;
    pos += static_cast<size_t>(m);
  }
  writeV2Record("SENS:GNSSCOV:", body);
}

void Neo3ProSensors::publishEcef() {
  if (!ecef_.valid) return;
  char body[600];
  int n = std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%.7g,%.7g,%.7g,%lld,%lld,%lld,%u",
      static_cast<unsigned long>(++ecef_sequence_), static_cast<unsigned long>(ecef_.received_ms),
      static_cast<unsigned>(ecef_.source_node_id), ecef_.velocity_xyz[0], ecef_.velocity_xyz[1],
      ecef_.velocity_xyz[2], static_cast<long long>(ecef_.position_xyz_mm[0]),
      static_cast<long long>(ecef_.position_xyz_mm[1]), static_cast<long long>(ecef_.position_xyz_mm[2]),
      static_cast<unsigned>(ecef_.covariance_len));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return;
  size_t pos = static_cast<size_t>(n);
  for (uint8_t i = 0U; i < ecef_.covariance_len && pos < sizeof(body); ++i) {
    const int m = std::snprintf(body + pos, sizeof(body) - pos, ",%.6g",
                                finiteOr(ecef_.covariance[i], -1.0F));
    if (m <= 0 || static_cast<size_t>(m) >= sizeof(body) - pos) return;
    pos += static_cast<size_t>(m);
  }
  writeV2Record("SENS:ECEF:", body);
}

void Neo3ProSensors::publishMag() {
  const float x = mag_.x_ga * 100.0F, y = mag_.y_ga * 100.0F, z = mag_.z_ga * 100.0F;
  const float norm = std::sqrt(x*x + y*y + z*z);
  char line[180];
  std::snprintf(line, sizeof(line), "SENS:MAG:%lu,%lu,%.3f,%.3f,%.3f,%.3f,1",
      static_cast<unsigned long>(++mag_sequence_), static_cast<unsigned long>(mag_.received_ms), x, y, z, norm);
  writeUsbLine(line); // compatibility stream; MAGPRO below is CRC protected
}

void Neo3ProSensors::publishMagPro() {
  char body[360];
  int n = std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u,%u,%.6g,%.6g,%.6g,%u",
      static_cast<unsigned long>(++mag_pro_sequence_), static_cast<unsigned long>(mag_.received_ms),
      static_cast<unsigned>(mag_.source_node_id), static_cast<unsigned>(mag_.message_type),
      static_cast<unsigned>(mag_.sensor_id), mag_.x_ga * 100.0F, mag_.y_ga * 100.0F,
      mag_.z_ga * 100.0F, static_cast<unsigned>(mag_.covariance_len));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return;
  size_t pos = static_cast<size_t>(n);
  for (uint8_t i = 0U; i < mag_.covariance_len; ++i) {
    // DSDL covariance unit is Gauss^2. Convert to Tesla^2 for direct ROS use.
    const float cov_t2 = mag_.covariance[i] * 1.0e-8F;
    const int m = std::snprintf(body + pos, sizeof(body) - pos, ",%.8g", finiteOr(cov_t2, -1.0F));
    if (m <= 0 || static_cast<size_t>(m) >= sizeof(body) - pos) return;
    pos += static_cast<size_t>(m);
  }
  writeV2Record("SENS:MAGPRO:", body);
}

void Neo3ProSensors::publishBaroPressure() {
  char body[180];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%.8g,%.8g",
      static_cast<unsigned long>(++baro_pressure_sequence_), static_cast<unsigned long>(baro_.pressure_ms),
      static_cast<unsigned>(baro_.source_node_id), baro_.pressure_pa,
      finiteOr(baro_.pressure_variance, -1.0F));
  writeV2Record("SENS:BARO:", body);
}

void Neo3ProSensors::publishBaroTemperature() {
  char body[180];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%.8g,%.8g",
      static_cast<unsigned long>(++baro_temperature_sequence_), static_cast<unsigned long>(baro_.temperature_ms),
      static_cast<unsigned>(baro_.source_node_id), baro_.temperature_k,
      finiteOr(baro_.temperature_variance, -1.0F));
  writeV2Record("SENS:TEMP:", body);
}

void Neo3ProSensors::publishNodeStatus() {
  char body[180];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%lu,%u,%u,%u,%u",
      static_cast<unsigned long>(++node_sequence_), static_cast<unsigned long>(node_.received_ms),
      static_cast<unsigned>(node_.source_node_id), static_cast<unsigned long>(node_.uptime_sec),
      static_cast<unsigned>(node_.health), static_cast<unsigned>(node_.mode),
      static_cast<unsigned>(node_.sub_mode), static_cast<unsigned>(node_.vendor_status));
  writeV2Record("SENS:NODE:", body);
}

void Neo3ProSensors::publishGnssStatus() {
  char body[180];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u,%lu,%lu",
      static_cast<unsigned long>(++gnss_status_sequence_), static_cast<unsigned long>(gnss_status_.received_ms),
      static_cast<unsigned>(gnss_status_.source_node_id), gnss_status_.healthy ? 1U : 0U,
      static_cast<unsigned long>(gnss_status_.error_codes), static_cast<unsigned long>(gnss_status_.status_bits));
  writeV2Record("SENS:GNSSSTAT:", body);
}

void Neo3ProSensors::publishHeading() {
  char body[180];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u,%u,%.8g,%.8g",
      static_cast<unsigned long>(++heading_sequence_), static_cast<unsigned long>(heading_.received_ms),
      static_cast<unsigned>(heading_.source_node_id), heading_.valid ? 1U : 0U,
      heading_.accuracy_valid ? 1U : 0U, finiteOr(heading_.heading_rad, 0.0F),
      finiteOr(heading_.accuracy_rad, -1.0F));
  writeV2Record("SENS:GNSSHEAD:", body);
}

void Neo3ProSensors::publishButton(bool pressed) {
  char body[160];
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u,%u,%u",
      static_cast<unsigned long>(++button_sequence_), static_cast<unsigned long>(HAL_GetTick()),
      static_cast<unsigned>(button_.source_node_id), static_cast<unsigned>(button_.button),
      static_cast<unsigned>(button_.press_time), pressed ? 1U : 0U);
  writeV2Record("SENS:BUTTON:", body);
}

bool Neo3ProSensors::magOk() const {
  return mag_.seen && static_cast<uint32_t>(HAL_GetTick() - mag_.received_ms) <= MAG_STALE_MS;
}

uint32_t Neo3ProSensors::lastCanFrameAgeMs(uint32_t now_ms) const {
  return last_raw_frame_ms_ == 0U ? 0xFFFFFFFFUL : static_cast<uint32_t>(now_ms - last_raw_frame_ms_);
}

void Neo3ProSensors::publishHardware(bool force) {
  const uint32_t now = HAL_GetTick();
  if (!force && static_cast<uint32_t>(now - last_hw_publish_ms_) < HW_PUBLISH_MS) return;
  last_hw_publish_ms_ = now;
  const bool gnss_alive = gnss_.seen && static_cast<uint32_t>(now - gnss_.received_ms) <= GNSS_STALE_MS;
  const bool gnss_ready = gnss_alive && gnss_.status >= 2U;
  const bool pressed = safetyPressed();
  const uint32_t seq = ++hw_sequence_;
  char body[260];
  // Legacy heartbeat retained for HMI/host compatibility. The safety field is
  // now real DroneCAN Button freshness, never a hardcoded false value.
  std::snprintf(body, sizeof(body), "%lu,%lu,%u,%u,%u,%u,0,%lu",
      static_cast<unsigned long>(seq), static_cast<unsigned long>(now), gnss_alive ? 1U : 0U,
      gnss_ready ? 1U : 0U, magOk() ? 1U : 0U, pressed ? 1U : 0U,
      static_cast<unsigned long>(recoveries_));
  char legacy_line[320];
  std::snprintf(legacy_line, sizeof(legacy_line), "SENS:HW:%s", body);
  writeUsbLine(legacy_line);

  uint8_t tec = 0U, rec = 0U, eflg = 0U;
  if (Board_SpiOwner() == BoardSpiOwner::NONE) {
    (void)mcpRead(REG_TEC, &tec); (void)mcpRead(REG_REC, &rec); (void)mcpRead(REG_EFLG, &eflg);
  }
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u",
      static_cast<unsigned long>(seq), static_cast<unsigned long>(now), can_ok_ ? 1U : 0U,
      static_cast<unsigned>(active_osc_mhz_), static_cast<unsigned>(primary_node_id_),
      static_cast<unsigned long>(raw_frames_), static_cast<unsigned long>(dronecan_transfers_),
      static_cast<unsigned long>(accepted_transfers_), static_cast<unsigned long>(foreign_node_drops_),
      static_cast<unsigned long>(spi_errors_), static_cast<unsigned long>(can_decode_errors_),
      static_cast<unsigned long>(can_overflows_), static_cast<unsigned long>(recoveries_),
      static_cast<unsigned long>(Board_SpiContentionCount()), static_cast<unsigned>(tec),
      static_cast<unsigned>(rec), static_cast<unsigned>(eflg));
  writeV2Record("SENS:HWPRO:", body);
}

bool Neo3ProSensors::handleHostCommand(const char *command) {
  if (!command) return false;
  if (std::strcmp(command, "NEO:STATUS") == 0 || std::strcmp(command, "NEO:CAN:STATUS") == 0) {
    publishHardware(true); return true;
  }
  if (std::strcmp(command, "NEO:CAN:RECOVER") == 0) {
    recoverCan(); publishHardware(true); return true;
  }
  return false;
}

#endif
