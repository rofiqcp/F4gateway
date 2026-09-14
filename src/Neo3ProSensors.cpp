#ifdef NEO3PRO

#include "Neo3ProSensors.h"
#include "BoardSupport.h"
#include "UsbCdcPort.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr uint16_t DTID_ALLOCATION = 1U;
constexpr uint64_t SIG_ALLOCATION = 0x0B2A812620A11D40ULL;
constexpr uint8_t SID_GET_NODE_INFO = 1U;
constexpr uint64_t SIG_GET_NODE_INFO = 0xEE468A8121C46A9EULL;
constexpr uint8_t SID_PARAM_GETSET = 11U;
constexpr uint64_t SIG_PARAM_GETSET = 0xA7B622F939D1A4D5ULL;
constexpr uint16_t DTID_NODE_STATUS = 341U;
constexpr uint64_t SIG_NODE_STATUS = 0x0F0868D0C1A7C6F1ULL;
constexpr uint16_t DTID_MAG1 = 1001U;
constexpr uint64_t SIG_MAG1 = 0xE2A7D4A9460BC2F2ULL;
constexpr uint16_t DTID_MAG2 = 1002U;
constexpr uint64_t SIG_MAG2 = 0xB6AC0C442430297EULL;
constexpr uint16_t DTID_MAG_HIRES = 1043U;
constexpr uint64_t SIG_MAG_HIRES = 0x3053EBE3D750286FULL;
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
constexpr uint16_t DTID_LIGHTS_COMMAND = 1081U;
constexpr uint64_t SIG_LIGHTS_COMMAND = 0x2031D93C8BDD1EC4ULL;
constexpr uint8_t DRONECAN_HOST_NODE_ID = 124U;
constexpr uint8_t DRONECAN_DEFAULT_ALLOCATED_NODE_ID = 125U;
constexpr uint16_t CUAV_GPS_APJ_BOARD_ID = 1001U;
constexpr uint32_t NODE_INFO_RETRY_MS = 1000U;
constexpr uint32_t PARAM_RETRY_MS = 500U;
// ArduPilot CANIface keeps frames pending until a real transport deadline; 1.5 ms
// was too aggressive for an external MCP2515 mailbox under arbitration/load.
// Keep the physical mailbox bounded, but long enough for normal CAN arbitration,
// and keep service transfers alive long enough for AP_Periph to answer.
constexpr uint32_t CAN_TX_HW_DEADLINE_US = 20000U;
constexpr uint64_t CAN_TX_QUEUE_DEADLINE_US = 250000ULL;
constexpr uint32_t CAN_TX_BACKOFF_BASE_MS = 25U;
constexpr uint32_t CAN_TX_BACKOFF_MAX_MS = 400U;
constexpr uint32_t CAN_HEALTH_CHECK_MS = 100U;
constexpr uint32_t CAN_RECOVERY_COOLDOWN_MS = 750U;
constexpr uint32_t CAN_NODE_STALE_MS = 5000U;
constexpr uint32_t CAN_RX_REALTIME_BUDGET_US = 700U;
constexpr uint32_t CAN_RX_MAIN_BUDGET_US = 2200U;
constexpr int64_t NEO3PRO_LED_BRIGHTNESS_TARGET = 30;
// Decouple DroneCAN RX from USB formatting/queueing. RX always consumes every
// transfer; USB publishes the latest coherent state at bounded rates.
constexpr uint32_t USB_MAG_PERIOD_MS = 40U;
constexpr uint32_t USB_BARO_PERIOD_MS = 100U;
constexpr uint32_t USB_TEMP_PERIOD_MS = 500U;
constexpr uint32_t USB_NODE_PERIOD_MS = 500U;
constexpr uint32_t USB_GNSS_META_PERIOD_MS = 100U;
constexpr uint32_t USB_GNSSSTAT_PERIOD_MS = 200U;
constexpr uint32_t USB_HEADING_PERIOD_MS = 100U;
constexpr uint32_t DIRTY_GNSS_FIX = 1UL << 0;
constexpr uint32_t DIRTY_GNSS_META = 1UL << 1;
constexpr uint32_t DIRTY_ECEF = 1UL << 2;
constexpr uint32_t DIRTY_MAG = 1UL << 3;
constexpr uint32_t DIRTY_BARO = 1UL << 4;
constexpr uint32_t DIRTY_TEMP = 1UL << 5;
constexpr uint32_t DIRTY_NODE = 1UL << 6;
constexpr uint32_t DIRTY_GNSSSTAT = 1UL << 7;
constexpr uint32_t DIRTY_HEADING = 1UL << 8;
constexpr uint32_t DIRTY_BUTTON = 1UL << 9;
constexpr uint32_t DIRTY_NODEINFO = 1UL << 10;
constexpr uint32_t DIRTY_PARAM = 1UL << 11;
constexpr uint32_t DIRTY_DNA = 1UL << 12;
constexpr uint32_t DIRTY_PEER_HEALTH = 1UL << 13;
constexpr uint32_t DIRTY_DNA_SERVER = 1UL << 14;
constexpr uint32_t CAN_RX_DECODE_BUDGET_US = 3000U;
constexpr uint32_t CAN_RX_IRQ_BUDGET_US = 180U;
constexpr uint8_t CAN_RX_IRQ_FRAME_BUDGET = 6U;
Neo3ProSensors *g_neo3pro_irq_owner = nullptr;
void Neo3ProMcpIrqThunk() {
  if (g_neo3pro_irq_owner != nullptr) g_neo3pro_irq_owner->irqFastDrain();
}

constexpr uint8_t MCP_RESET = 0xC0U;
constexpr uint8_t MCP_READ = 0x03U;
constexpr uint8_t MCP_WRITE = 0x02U;
constexpr uint8_t MCP_BITMOD = 0x05U;
constexpr uint8_t MCP_READ_STATUS = 0xA0U;
constexpr uint8_t MCP_READ_RX0 = 0x90U;
constexpr uint8_t MCP_READ_RX1 = 0x94U;
constexpr uint8_t MCP_RTS_TX0 = 0x81U;
constexpr uint8_t REG_RXF0SIDH = 0x00U;
constexpr uint8_t REG_RXF1SIDH = 0x04U;
constexpr uint8_t REG_RXF2SIDH = 0x08U;
constexpr uint8_t REG_RXF3SIDH = 0x10U;
constexpr uint8_t REG_RXF4SIDH = 0x14U;
constexpr uint8_t REG_RXF5SIDH = 0x18U;
constexpr uint8_t REG_RXM0SIDH = 0x20U;
constexpr uint8_t REG_RXM1SIDH = 0x24U;
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
constexpr uint8_t REG_TXB0CTRL = 0x30U;
constexpr uint8_t REG_TXB0SIDH = 0x31U;
constexpr uint8_t TXREQ = 0x08U;
constexpr uint8_t CANCTRL_ABAT = 0x10U;
constexpr uint8_t EFLG_EWARN = 0x01U;
constexpr uint8_t EFLG_RXWAR = 0x02U;
constexpr uint8_t EFLG_TXWAR = 0x04U;
constexpr uint8_t EFLG_RXEP = 0x08U;
constexpr uint8_t EFLG_TXEP = 0x10U;
constexpr uint8_t EFLG_TXBO = 0x20U;
constexpr uint8_t MODE_MASK = 0xE0U;
constexpr uint8_t MODE_CONFIG = 0x80U;
constexpr uint8_t MODE_NORMAL = 0x00U;
constexpr uint8_t RX0IF = 0x01U;
constexpr uint8_t RX1IF = 0x02U;
constexpr uint8_t ERRIF = 0x20U;
constexpr uint8_t RX0OVR = 0x40U;
constexpr uint8_t RX1OVR = 0x80U;
constexpr uint32_t PROFILE_PROBE_MS = 1400U;
constexpr uint32_t HW_PUBLISH_MS = 1000U;
constexpr uint32_t GNSS_STALE_MS = 1500U;
constexpr uint32_t MAG_STALE_MS = 800U;
constexpr uint32_t BUTTON_STALE_MS = 250U; // AP_Periph transmits safety Button at 10 Hz while held
constexpr uint8_t MAX_RX_PER_POLL = 96U;
constexpr uint8_t MAX_RX_REALTIME = 24U;
constexpr uint64_t GPS_WEEK_USEC = 604800000000ULL;
#ifndef NEO3PRO_EXPECTED_NODE_ID
#define NEO3PRO_EXPECTED_NODE_ID 0
#endif
static_assert(NEO3PRO_EXPECTED_NODE_ID >= 0 && NEO3PRO_EXPECTED_NODE_ID <= 127,
              "NEO3PRO_EXPECTED_NODE_ID must be 0(auto) or 1..127");

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

void formatU64(char *dst, size_t n, uint64_t value) {
  if (!dst || n == 0U) return;
  char rev[24]; size_t k = 0U;
  do { rev[k++] = static_cast<char>('0' + (value % 10U)); value /= 10U; } while (value && k < sizeof(rev));
  size_t out = 0U;
  while (k > 0U && out + 1U < n) dst[out++] = rev[--k];
  dst[out] = '\0';
}

void formatI64(char *dst, size_t n, int64_t value) {
  if (!dst || n == 0U) return;
  if (value < 0) {
    if (n < 2U) { dst[0] = '\0'; return; }
    dst[0] = '-';
    const uint64_t mag = static_cast<uint64_t>(-(value + 1)) + 1U;
    formatU64(dst + 1U, n - 1U, mag);
  } else {
    formatU64(dst, n, static_cast<uint64_t>(value));
  }
}
} // namespace

bool Neo3ProSensors::begin() {
  canardInit(&canard_, canard_pool_, sizeof(canard_pool_), &Neo3ProSensors::onTransfer,
             &Neo3ProSensors::shouldAccept, this);
  g_neo3pro_irq_owner = this;
  Board_SetMcpFastIrqCallback(&Neo3ProMcpIrqThunk);
  // The F4 gateway is a real DroneCAN node. Keep the server itself registered in
  // the same persistent database as ArduPilot DNA so allocator ID 124 can never
  // be handed to another node.
  canardSetLocalNodeID(&canard_, DRONECAN_HOST_NODE_ID);
  uint8_t own_uid[16]{};
  buildOwnDnaUid(own_uid);
  if (!dna_db_.begin(DRONECAN_HOST_NODE_ID, own_uid, sizeof(own_uid))) ++dna_storage_faults_;
  dna_seen_[DRONECAN_HOST_NODE_ID] = true;
  dna_verified_[DRONECAN_HOST_NODE_ID] = true;
  dna_healthy_[DRONECAN_HOST_NODE_ID] = true;
  dna_last_seen_ms_[DRONECAN_HOST_NODE_ID] = HAL_GetTick();
  dna_server_state_ = DnaServerState::HEALTHY;
  dna_curr_verifying_node_ = DRONECAN_HOST_NODE_ID;
  dna_nodeinfo_response_received_ = true;
  peer_state_since_ms_ = HAL_GetTick();
  telemetry_dirty_ |= DIRTY_PEER_HEALTH | DIRTY_DNA_SERVER;
  probe_index_ = 0U;
#if NEO3PRO_EXPECTED_NODE_ID > 0
  primary_node_id_ = static_cast<uint8_t>(NEO3PRO_EXPECTED_NODE_ID);
#endif
  return initMcp(8U);
}

void Neo3ProSensors::buildOwnDnaUid(uint8_t out[16]) const {
  if (!out) return;
  std::memset(out, 0, 16U);
  // STM32F411 96-bit factory unique ID, extended with the gateway board ID to
  // form a stable 16-byte DroneCAN server UID.
  const auto *uid = reinterpret_cast<const uint8_t *>(0x1FFF7A10UL);
  std::memcpy(out, uid, 12U);
  const uint32_t board = 0xF411CE01UL;
  std::memcpy(out + 12U, &board, sizeof(board));
}

bool Neo3ProSensors::dnaNodeSeen(uint8_t node_id) const {
  return node_id > 0U && node_id <= DroneCanDnaDatabase::kMaxNodeId && dna_seen_[node_id];
}

bool Neo3ProSensors::dnaNodeVerified(uint8_t node_id) const {
  return node_id > 0U && node_id <= DroneCanDnaDatabase::kMaxNodeId && dna_verified_[node_id];
}

bool Neo3ProSensors::dnaNodeHealthy(uint8_t node_id) const {
  return node_id > 0U && node_id <= DroneCanDnaDatabase::kMaxNodeId && dna_healthy_[node_id];
}

void Neo3ProSensors::recomputeDnaServerState() {
  // Duplicate-node is a hard prearm-equivalent failure in ArduPilot and remains
  // latched for this boot because two devices can answer the same service ID.
  if (dna_server_state_ == DnaServerState::DUPLICATE_NODES) return;
  for (uint8_t id = 1U; id <= DroneCanDnaDatabase::kMaxNodeId; ++id) {
    if (dna_seen_[id] && !dna_healthy_[id]) {
      dna_server_state_ = DnaServerState::NODE_STATUS_UNHEALTHY;
      dna_fault_node_id_ = id;
      telemetry_dirty_ |= DIRTY_DNA_SERVER | DIRTY_PEER_HEALTH;
      return;
    }
  }
  // ArduPilot only returns from NODE_STATUS_UNHEALTHY to HEALTHY when the
  // healthy and verified node sets agree. Preserve that fail-safe behavior.
  if (dna_server_state_ == DnaServerState::NODE_STATUS_UNHEALTHY) {
    for (uint8_t id = 1U; id <= DroneCanDnaDatabase::kMaxNodeId; ++id) {
      if (dna_seen_[id] && !dna_verified_[id]) {
        telemetry_dirty_ |= DIRTY_DNA_SERVER;
        return;
      }
    }
  }
  dna_server_state_ = DnaServerState::HEALTHY;
  dna_fault_node_id_ = 0U;
  telemetry_dirty_ |= DIRTY_DNA_SERVER;
}

void Neo3ProSensors::markDnaNodeStatus(const NodeState &status) {
  const uint8_t id = status.source_node_id;
  if (id == 0U || id > DroneCanDnaDatabase::kMaxNodeId) return;
  dna_seen_[id] = true;
  dna_healthy_[id] = status.health == 0U && status.mode == 0U;
  dna_last_seen_ms_[id] = status.received_ms;
  dna_last_uptime_[id] = status.uptime_sec;
  // ArduPilot immediately begins GetNodeInfo verification for a seen node that
  // is not yet verified. Do not change authority until that response is valid.
  if (!dna_verified_[id] && id != DRONECAN_HOST_NODE_ID) {
    const bool primary_stale = primary_node_id_ == 0U || !dnaNodeSeen(primary_node_id_) ||
        static_cast<uint32_t>(status.received_ms - dna_last_seen_ms_[primary_node_id_]) > CAN_NODE_STALE_MS;
    if (discovery_node_id_ == 0U || discovery_node_id_ == id || primary_stale) {
      if (discovery_node_id_ != id) last_identity_request_ms_ = 0U;
      discovery_node_id_ = id;
    }
  }
  recomputeDnaServerState();
}

void Neo3ProSensors::serviceDnaVerification(uint32_t now_ms) {
  // Match AP_DroneCAN_DNA_Server::verify_nodes(): no more than one verification
  // request every 5 seconds, cycling all seen node IDs.
  if (static_cast<uint32_t>(now_ms - dna_last_verification_request_ms_) < 5000U) return;
  if (dna_curr_verifying_node_ == DRONECAN_HOST_NODE_ID) dna_nodeinfo_response_received_ = true;
  if (dna_curr_verifying_node_ != 0U && dna_curr_verifying_node_ != DRONECAN_HOST_NODE_ID &&
      !dna_nodeinfo_response_received_) {
    // A missed GetNodeInfo response is a liveness/service failure, not proof that
    // the persisted identity became invalid. Sensor authority is already revoked
    // by NodeStatus freshness/health. Keep the verified UID binding so a peer that
    // returns can recover immediately; uptime rollback below explicitly revokes it.
    ++dna_verification_failures_;
    telemetry_dirty_ |= DIRTY_DNA_SERVER | DIRTY_PEER_HEALTH;
    recomputeDnaServerState();
  }
  dna_last_verification_request_ms_ = now_ms;
  uint8_t next = dna_curr_verifying_node_;
  bool found = false;
  for (uint16_t n = 0U; n <= DroneCanDnaDatabase::kMaxNodeId; ++n) {
    next = static_cast<uint8_t>((next + 1U) % (DroneCanDnaDatabase::kMaxNodeId + 1U));
    if (next == 0U || next == DRONECAN_HOST_NODE_ID) continue;
    // Never emit maintenance/service traffic to a stale node. The MCP2515 will
    // otherwise retry an unacknowledged request, increasing TEC while the peer is
    // physically absent. Verification resumes automatically on fresh NodeStatus.
    const bool fresh = dna_seen_[next] &&
        static_cast<uint32_t>(now_ms - dna_last_seen_ms_[next]) <= CAN_NODE_STALE_MS;
    if (fresh) { found = true; break; }
  }
  if (!found) {
    dna_curr_verifying_node_ = DRONECAN_HOST_NODE_ID;
    dna_nodeinfo_response_received_ = true;
    recomputeDnaServerState();
    return;
  }
  dna_curr_verifying_node_ = next;
  dna_nodeinfo_response_received_ = false;
  // Registered nodes are periodically re-verified; unregistered-but-seen nodes
  // are also queried so their first NodeInfo can populate the persistent DB.
  if (!requestGetNodeInfo(next)) {
    // Leave the response flag false so diagnostics count the missed service on
    // the next verification pass. Persistent UID verification is not revoked by
    // this liveness failure; NodeStatus freshness controls sensor authority.
  }
}

bool Neo3ProSensors::mcpTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len) {
  if (!tx || len == 0U) return false;
  // Serialize main-context HAL SPI against the EXTI fast RX copier. The short
  // critical section only claims ownership; interrupts are enabled during HAL
  // transfer so UART/USB/SysTick continue normally. If MCP INT arrives while
  // SPI2 is busy, the ISR leaves the pending latch set for immediate follow-up.
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (mcp_spi_busy_) {
    if (primask == 0U) __enable_irq();
    return false;
  }
  mcp_spi_busy_ = true;
  if (primask == 0U) __enable_irq();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_StatusTypeDef st = rx
      ? HAL_SPI_TransmitReceive(&hspi2, const_cast<uint8_t *>(tx), rx, len, 2U)
      : HAL_SPI_Transmit(&hspi2, const_cast<uint8_t *>(tx), len, 2U);
  const uint32_t busy_start = DWT->CYCCNT;
  const uint32_t busy_limit = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U) * 500U;
  while ((SPI2->SR & SPI_SR_BSY) != 0U &&
         static_cast<uint32_t>(DWT->CYCCNT - busy_start) < busy_limit) {}
  if ((SPI2->SR & SPI_SR_BSY) != 0U) st = HAL_TIMEOUT;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  primask = __get_PRIMASK();
  __disable_irq();
  mcp_spi_busy_ = false;
  if (primask == 0U) __enable_irq();

  if (st != HAL_OK) {
    ++spi_errors_;
    can_ok_ = false;
    can_recovery_pending_ = true;
  }
  return st == HAL_OK;
}

bool Neo3ProSensors::mcpFastTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len,
                                     uint32_t deadline_cycles) {
  if (!tx || len == 0U || !can_ok_ || mcp_spi_busy_) return false;
  mcp_spi_busy_ = true; // EXTI0 priority: no same-priority re-entry on this IRQ.
  const uint32_t started = DWT->CYCCNT;
  auto timedOut = [&]() -> bool {
    return static_cast<uint32_t>(DWT->CYCCNT - started) >= deadline_cycles;
  };
  GPIOB->BSRR = static_cast<uint32_t>(GPIO_PIN_12) << 16U; // CS low
  bool ok = true;
  for (uint16_t i = 0U; i < len; ++i) {
    while ((SPI2->SR & SPI_SR_TXE) == 0U) { if (timedOut()) { ok = false; break; } }
    if (!ok) break;
    *reinterpret_cast<volatile uint8_t *>(&SPI2->DR) = tx[i];
    while ((SPI2->SR & SPI_SR_RXNE) == 0U) { if (timedOut()) { ok = false; break; } }
    if (!ok) break;
    const uint8_t v = *reinterpret_cast<volatile uint8_t *>(&SPI2->DR);
    if (rx) rx[i] = v;
  }
  while (ok && (SPI2->SR & SPI_SR_BSY) != 0U) { if (timedOut()) { ok = false; break; } }
  GPIOB->BSRR = GPIO_PIN_12; // CS high; READ_RX_BUFFER clears RXnIF here.
  mcp_spi_busy_ = false;
  if (!ok) {
    // Never reset/reconfigure from ISR. Main context observes the pending INT and
    // health counters; a genuine SPI/controller fault is handled by normal recovery.
    ++spi_errors_;
  }
  return ok;
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

bool Neo3ProSensors::mcpReadStatus(uint8_t *status) {
  if (!status) return false;
  const uint8_t tx[2] = {MCP_READ_STATUS, 0xFFU};
  uint8_t rx[2]{};
  if (!mcpTransfer(tx, rx, sizeof(tx))) return false;
  *status = rx[1];
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
  // Mark unavailable before reset/config. Board_RealtimeDelayMs() invokes the
  // cooperative callback, so pollRealtime() must not re-enter MCP SPI while
  // the controller is in CONFIG/reset state.
  can_ok_ = false;
  // Production hardware contract: MCP2515 crystal is 8 MHz.
  // Ignore any legacy caller/profile value and always configure 8 MHz timing.
  osc_mhz = 8U;
  Board_SpiDeselectAll();
  const uint8_t reset = MCP_RESET;
  if (!mcpTransfer(&reset, nullptr, 1U)) { can_ok_ = false; return false; }
  Board_RealtimeDelayMs(3U);
  if (!mcpSetMode(MODE_CONFIG)) { can_ok_ = false; return false; }

  // DroneCAN classic CAN runs at 1 Mbit/s. Production MCP2515 crystal is 8 MHz.
  // BRP=0 gives TQ=250 ns; Sync+Prop+PS1+PS2 = 4 TQ -> 1 us/bit.
  // Sample point is 75%. SAM=0 (single sample) is used for this high-speed bus.
  const uint8_t cnf1 = 0x00U, cnf2 = 0x80U, cnf3 = 0x80U;
  if (!mcpWrite(REG_CNF1, cnf1) || !mcpWrite(REG_CNF2, cnf2) ||
      !mcpWrite(REG_CNF3, cnf3)) { can_ok_ = false; return false; }

  // Hardware-filter standard 11-bit traffic before it reaches the two tiny RX
  // buffers. DroneCAN v0 uses extended 29-bit identifiers. MIDE=1 in each mask
  // makes IDE participate in filtering, while all identifier mask bits remain
  // zero; EXIDE=1 in every filter therefore means "accept any EXTENDED frame".
  // RXB0 rollover to RXB1 protects short shared-SPI/HMI critical sections.
  const uint8_t filter_bases[] = {REG_RXF0SIDH, REG_RXF1SIDH, REG_RXF2SIDH,
                                  REG_RXF3SIDH, REG_RXF4SIDH, REG_RXF5SIDH};
  const uint8_t mask_bases[] = {REG_RXM0SIDH, REG_RXM1SIDH};
  for (uint8_t base : mask_bases) {
    if (!mcpWrite(base + 0U, 0x00U) || !mcpWrite(base + 1U, 0x08U) ||
        !mcpWrite(base + 2U, 0x00U) || !mcpWrite(base + 3U, 0x00U)) {
      can_ok_ = false; return false;
    }
  }
  for (uint8_t base : filter_bases) {
    if (!mcpWrite(base + 0U, 0x00U) || !mcpWrite(base + 1U, 0x08U) ||
        !mcpWrite(base + 2U, 0x00U) || !mcpWrite(base + 3U, 0x00U)) {
      can_ok_ = false; return false;
    }
  }
  if (!mcpWrite(REG_RXB0CTRL, 0x04U) || !mcpWrite(REG_RXB1CTRL, 0x00U) ||
      !mcpWrite(REG_CANINTE, RX0IF | RX1IF | ERRIF) ||
      !mcpWrite(REG_CANINTF, 0x00U) || !mcpWrite(REG_EFLG, 0x00U) ||
      !mcpSetMode(MODE_NORMAL)) { can_ok_ = false; return false; }
  // DroneCAN uses normal CAN arbitration/retransmission semantics (MCP2515 OSM=0).
  // Individual host commands remain bounded by writeOneCanFrame() and are gated
  // on a recently verified NEO node, so an unplugged peer cannot poison the bus.
  (void)mcpBitModify(REG_CANCTRL, 0x08U, 0x00U);

  active_osc_mhz_ = osc_mhz;
  profile_started_ms_ = HAL_GetTick();
  last_raw_frame_ms_ = 0U;
  can_ok_ = true;
  return true;
}

void Neo3ProSensors::maybeProbeOscillator(uint32_t now_ms) {
  (void)now_ms;
  // Hardware qualification confirmed this MCP2515 board uses an 8 MHz crystal.
  // Keep bit timing fixed at 1 Mbit/s using the 8 MHz profile; never probe/toggle
  // to 16 MHz because that destroys DroneCAN multi-frame continuity.
  oscillator_locked_ = true;
}

bool Neo3ProSensors::parseAndQueueMcpRx(const uint8_t *rx, uint64_t timestamp_us) {
  if (!rx) return false;
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
  // Freshness/failsafe is intentionally on the HAL millisecond clock. The exact
  // DWT timestamp stays in RawCanSlot for libcanard and latency analysis; never
  // compare different clock epochs for peer freshness.
  last_raw_frame_ms_ = HAL_GetTick();
  if (!extended) { ++raw_can_nonextended_; return true; }

  const bool service = ((id >> 7U) & 0x1U) != 0U;
  const uint8_t raw_source = static_cast<uint8_t>(id & 0x7FU);
  const uint16_t raw_dtid = service ? static_cast<uint16_t>((id >> 16U) & 0xFFU)
      : (raw_source == 0U ? static_cast<uint16_t>((id >> 8U) & 0x03U)
                          : static_cast<uint16_t>((id >> 8U) & 0xFFFFU));
  if (!service && raw_dtid == DTID_ALLOCATION) ++raw_allocation_frames_;
  else if (!service && raw_dtid == DTID_FIX2) ++raw_fix2_frames_;
  else if (!service && raw_dtid == DTID_NODE_STATUS) ++raw_node_frames_;
  else if (!service && (raw_dtid == DTID_MAG1 || raw_dtid == DTID_MAG2 || raw_dtid == DTID_MAG_HIRES)) ++raw_mag_frames_;
  else ++raw_other_frames_;
  return enqueueRawCan(id, dlc, &rx[6], timestamp_us);
}

bool Neo3ProSensors::readOneCanFrame(uint8_t buffer_index) {
  uint8_t tx[14]{};
  uint8_t rx[14]{};
  tx[0] = buffer_index == 0U ? MCP_READ_RX0 : MCP_READ_RX1;
  for (uint8_t i = 1U; i < sizeof(tx); ++i) tx[i] = 0xFFU;
  if (!mcpTransfer(tx, rx, sizeof(tx))) return false;
  return parseAndQueueMcpRx(rx, Board_MonotonicMicros64());
}

bool Neo3ProSensors::mcpFastReadOneCanFrame(uint8_t buffer_index, uint32_t deadline_cycles) {
  uint8_t tx[14]{};
  uint8_t rx[14]{};
  tx[0] = buffer_index == 0U ? MCP_READ_RX0 : MCP_READ_RX1;
  for (uint8_t i = 1U; i < sizeof(tx); ++i) tx[i] = 0xFFU;
  if (!mcpFastTransfer(tx, rx, sizeof(tx), deadline_cycles)) return false;
  if (!parseAndQueueMcpRx(rx, Board_MonotonicMicros64())) return false;
  ++can_irq_fast_frames_;
  return true;
}

void Neo3ProSensors::irqFastDrain() {
  if (!can_ok_) return;
  if (mcp_spi_busy_) { ++can_irq_spi_busy_deferrals_; return; }
  const uint32_t cycles_per_us = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  const uint32_t start = DWT->CYCCNT;
  const uint32_t total_budget = cycles_per_us * CAN_RX_IRQ_BUDGET_US;
  uint8_t frames = 0U;
  while (frames < CAN_RX_IRQ_FRAME_BUDGET) {
    const uint32_t elapsed = static_cast<uint32_t>(DWT->CYCCNT - start);
    if (elapsed >= total_budget) { ++can_irq_fast_budget_exhaustions_; break; }
    uint8_t tx[2] = {MCP_READ_STATUS, 0xFFU};
    uint8_t rx[2]{};
    if (!mcpFastTransfer(tx, rx, sizeof(tx), total_budget - elapsed)) break;
    const uint8_t st = rx[1];
    bool consumed = false;
    if ((st & RX0IF) != 0U && frames < CAN_RX_IRQ_FRAME_BUDGET) {
      const uint32_t rem = total_budget - static_cast<uint32_t>(DWT->CYCCNT - start);
      if (rem == 0U || !mcpFastReadOneCanFrame(0U, rem)) break;
      ++frames; consumed = true;
    }
    if ((st & RX1IF) != 0U && frames < CAN_RX_IRQ_FRAME_BUDGET) {
      const uint32_t elapsed2 = static_cast<uint32_t>(DWT->CYCCNT - start);
      if (elapsed2 >= total_budget || !mcpFastReadOneCanFrame(1U, total_budget - elapsed2)) break;
      ++frames; consumed = true;
    }
    if (!consumed) break; // ERRIF/TXIF stays for main-context health service.
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_RESET) break;
  }
  const uint32_t us = static_cast<uint32_t>(DWT->CYCCNT - start) / cycles_per_us;
  if (us > can_irq_fast_max_us_) can_irq_fast_max_us_ = us;
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_RESET) Board_McpIntClear();
}

bool Neo3ProSensors::enqueueRawCan(uint32_t id, uint8_t dlc, const uint8_t *data,
                                    uint64_t timestamp_us) {
  if (dlc > 8U || (dlc > 0U && data == nullptr)) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (raw_can_count_ >= RAW_CAN_QUEUE_CAPACITY) {
    ++raw_can_sw_overflows_;
    if (primask == 0U) __enable_irq();
    return false;
  }
  RawCanSlot &slot = raw_can_queue_[raw_can_head_];
  slot.id = id; slot.timestamp_us = timestamp_us; slot.dlc = dlc;
  if (dlc > 0U) std::memcpy(slot.data, data, dlc);
  if (dlc < sizeof(slot.data)) std::memset(slot.data + dlc, 0, sizeof(slot.data) - dlc);
  raw_can_head_ = static_cast<uint16_t>((raw_can_head_ + 1U) % RAW_CAN_QUEUE_CAPACITY);
  ++raw_can_count_;
  ++raw_can_enqueued_;
  if (raw_can_count_ > raw_can_hwm_) raw_can_hwm_ = raw_can_count_;
  if (primask == 0U) __enable_irq();
  return true;
}

void Neo3ProSensors::processRawCanFrame(const RawCanSlot &slot) {
  CanardCANFrame frame{};
  frame.id = slot.id | CANARD_CAN_FRAME_EFF;
  frame.data_len = slot.dlc;
  frame.iface_id = 0U;
  std::memcpy(frame.data, slot.data, slot.dlc);
  const int16_t res = canardHandleRxFrame(&canard_, &frame,
      slot.timestamp_us);
  ++raw_can_processed_;
  if (res >= 0) return;

  // Match ArduPilot protocol statistics. MISSED_START from a data type we do not
  // subscribe to is filtering, not corruption/resynchronization failure.
  if (-res == CANARD_ERROR_RX_MISSED_START) {
    const bool service = ((slot.id >> 7U) & 0x1U) != 0U;
    const uint8_t source = static_cast<uint8_t>(slot.id & 0x7FU);
    const uint16_t dtid = service ? static_cast<uint16_t>((slot.id >> 16U) & 0xFFU)
        : (source == 0U ? static_cast<uint16_t>((slot.id >> 8U) & 0x03U)
                        : static_cast<uint16_t>((slot.id >> 8U) & 0xFFFFU));
    bool known = false;
    if (!service) {
      switch (dtid) {
        case DTID_ALLOCATION: case DTID_NODE_STATUS: case DTID_MAG1: case DTID_MAG2:
        case DTID_MAG_HIRES: case DTID_PRESSURE: case DTID_TEMPERATURE: case DTID_AUX:
        case DTID_FIX2: case DTID_BUTTON: case DTID_GNSS_HEADING: case DTID_GNSS_STATUS:
          known = true; break;
        default: break;
      }
    } else {
      known = dtid == SID_GET_NODE_INFO || dtid == SID_PARAM_GETSET;
    }
    if (!known) { ++can_rx_not_wanted_; return; }

    // Low-rate proof diagnostic: capture only genuine MISSED_START events for
    // data types/services this gateway subscribes to. This executes in main
    // context after the raw FIFO, never in the MCP2515 ISR.
    const uint8_t tail = slot.dlc > 0U ? slot.data[slot.dlc - 1U] : 0U;
    const uint64_t now_us = Board_MonotonicMicros64();
    const uint32_t queue_age_us = now_us >= slot.timestamp_us
        ? static_cast<uint32_t>(std::min<uint64_t>(0xFFFFFFFFULL, now_us - slot.timestamp_us)) : 0U;
    char miss_body[180];
    std::snprintf(miss_body, sizeof(miss_body),
        "%lu,%lu,%08lX,%u,%u,%u,%u,%u,%u,%u,%lu",
        static_cast<unsigned long>(can_rx_missed_start_ + 1U),
        static_cast<unsigned long>(HAL_GetTick()),
        static_cast<unsigned long>(slot.id),
        static_cast<unsigned>(source), static_cast<unsigned>(dtid), service ? 1U : 0U,
        (tail & 0x80U) != 0U ? 1U : 0U, (tail & 0x40U) != 0U ? 1U : 0U,
        (tail & 0x20U) != 0U ? 1U : 0U, static_cast<unsigned>(tail & 0x1FU),
        static_cast<unsigned long>(queue_age_us));
    writeV2Record("SENS:CANMISS:", miss_body);
  }

  switch (-res) {
    case CANARD_ERROR_RX_NOT_WANTED: ++can_rx_not_wanted_; break;
    case CANARD_ERROR_RX_WRONG_ADDRESS: ++can_rx_wrong_address_; break;
    case CANARD_ERROR_RX_MISSED_START: ++can_rx_missed_start_; ++can_rx_resync_events_; break;
    case CANARD_ERROR_RX_WRONG_TOGGLE: ++can_rx_wrong_toggle_; ++can_rx_resync_events_; break;
    case CANARD_ERROR_RX_UNEXPECTED_TID: ++can_rx_unexpected_tid_; ++can_rx_resync_events_; break;
    case CANARD_ERROR_RX_SHORT_FRAME: ++can_rx_short_frame_; ++can_decode_errors_; break;
    case CANARD_ERROR_RX_BAD_CRC: ++can_rx_bad_crc_; ++can_decode_errors_; break;
    case CANARD_ERROR_OUT_OF_MEMORY: ++can_rx_oom_; ++can_decode_errors_; break;
    case CANARD_ERROR_INTERNAL: ++can_rx_internal_; ++can_decode_errors_; break;
    default: ++can_rx_incompatible_; ++can_decode_errors_; break;
  }
}

void Neo3ProSensors::processRawCanQueue(uint16_t frame_budget, uint32_t time_budget_us) {
  const uint32_t started = DWT->CYCCNT;
  const uint32_t cycles_per_us = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  while (raw_can_count_ > 0U && frame_budget-- > 0U) {
    if (static_cast<uint32_t>(DWT->CYCCNT - started) >= cycles_per_us * time_budget_us) break;
    RawCanSlot slot{};
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (raw_can_count_ == 0U) {
      if (primask == 0U) __enable_irq();
      break;
    }
    slot = raw_can_queue_[raw_can_tail_];
    raw_can_tail_ = static_cast<uint16_t>((raw_can_tail_ + 1U) % RAW_CAN_QUEUE_CAPACITY);
    --raw_can_count_;
    ++raw_can_dequeued_;
    if (primask == 0U) __enable_irq();
    processRawCanFrame(slot);
  }
}

bool Neo3ProSensors::abortMcpTxBounded() {
  ++can_tx_abort_count_;
  mcp_tx_pending_ = false;
  mcp_tx_started_cycles_ = 0U;
  if (!mcpBitModify(REG_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT)) return false;
  bool cleared = false;
  const uint32_t started = HAL_GetTick();
  do {
    uint8_t st = 0U;
    if (!mcpRead(REG_TXB0CTRL, &st)) break;
    if ((st & TXREQ) == 0U) { cleared = true; break; }
  } while (static_cast<uint32_t>(HAL_GetTick() - started) < 2U);
  (void)mcpBitModify(REG_CANCTRL, CANCTRL_ABAT, 0U);
  (void)mcpBitModify(REG_TXB0CTRL, 0x70U, 0U);
  return cleared;
}

void Neo3ProSensors::clearCanardTxQueue() {
  while (canardPeekTxQueue(&canard_) != nullptr) canardPopTxQueue(&canard_);
}

bool Neo3ProSensors::canTxPermitted(uint32_t now_ms) const {
  return can_ok_ &&
         static_cast<int32_t>(now_ms - can_tx_backoff_until_ms_) >= 0;
}

void Neo3ProSensors::noteCanTxFailure(uint32_t now_ms) {
  if (can_tx_fail_streak_ < 7U) ++can_tx_fail_streak_;
  const uint8_t shift = std::min<uint8_t>(4U, static_cast<uint8_t>(can_tx_fail_streak_ - 1U));
  const uint32_t delay = std::min<uint32_t>(CAN_TX_BACKOFF_MAX_MS,
      CAN_TX_BACKOFF_BASE_MS << shift);
  can_tx_backoff_until_ms_ = now_ms + delay;
}

void Neo3ProSensors::serviceCanHealth(uint32_t now_ms) {
  serviceMcpTx();
  if (static_cast<uint32_t>(now_ms - last_can_health_ms_) < CAN_HEALTH_CHECK_MS) return;
  last_can_health_ms_ = now_ms;

  if (!can_ok_) {
    if (can_recovery_pending_ &&
        static_cast<uint32_t>(now_ms - last_can_recovery_ms_) >= CAN_RECOVERY_COOLDOWN_MS)
      recoverCan();
    return;
  }

  uint8_t tec = 0U, rec = 0U, eflg = 0U;
  if (!mcpRead(REG_TEC, &tec) || !mcpRead(REG_REC, &rec) || !mcpRead(REG_EFLG, &eflg)) {
    can_recovery_pending_ = true;
    can_ok_ = false;
    return;
  }
  can_health_tec_ = tec;
  can_health_rec_ = rec;
  can_health_eflg_ = eflg;

  if ((eflg & (RX0OVR | RX1OVR)) != 0U) {
    if (eflg & RX0OVR) ++can_rx0_overflows_;
    if (eflg & RX1OVR) ++can_rx1_overflows_;
    ++can_overflows_;
    (void)mcpBitModify(REG_EFLG, RX0OVR | RX1OVR, 0U);
  }

  // Match ArduPilot CANIface semantics: error-passive is diagnostic/backpressure,
  // not a controller-reset condition. Only an actual BUS-OFF boundary may trigger
  // controller recovery. TX mailbox deadlines independently abort no-ACK frames.
  if ((eflg & EFLG_TXEP) != 0U || tec >= 128U) {
    can_tx_backoff_until_ms_ = std::max<uint32_t>(can_tx_backoff_until_ms_,
        now_ms + CAN_TX_BACKOFF_BASE_MS);
  }
  if ((eflg & EFLG_TXBO) != 0U) {
    ++can_busoff_count_;
    (void)abortMcpTxBounded();
    clearCanardTxQueue();
    can_recovery_pending_ = true;
    can_tx_backoff_until_ms_ = now_ms + CAN_RECOVERY_COOLDOWN_MS;
    if (static_cast<uint32_t>(now_ms - last_can_recovery_ms_) >= CAN_RECOVERY_COOLDOWN_MS)
      recoverCan();
  }
}

bool Neo3ProSensors::writeOneCanFrame(const CanardCANFrame *frame) {
  if (!frame || (frame->id & CANARD_CAN_FRAME_EFF) == 0U ||
      frame->data_len == 0U || frame->data_len > 8U || mcp_tx_pending_) return false;
  const uint32_t now = HAL_GetTick();
  if (!canTxPermitted(now)) return false;

  uint8_t tec = 0U, eflg = 0U, ctrl = 0U;
  if (!mcpRead(REG_TEC, &tec) || !mcpRead(REG_EFLG, &eflg) ||
      !mcpRead(REG_TXB0CTRL, &ctrl)) {
    ++can_tx_errors_; noteCanTxFailure(now); return false;
  }
  if ((eflg & EFLG_TXBO) != 0U) {
    ++can_tx_errors_;
    noteCanTxFailure(now);
    can_recovery_pending_ = true;
    return false;
  }
  // Error-passive must NOT suppress every transmission. TEC can decrease only
  // after successful acknowledged frames, so a permanent TXEP/TEC>=128 reject
  // deadlocks recovery after a peer returns. Keep the normal bounded backoff and
  // hardware-mailbox deadline, but allow one real recovery probe when permitted.
  if ((eflg & EFLG_TXEP) != 0U || tec >= 128U)
    ++can_tx_passive_probe_count_;
  if ((ctrl & TXREQ) != 0U) {
    // Unexpected orphaned mailbox. Abort only on this fault path; normal
    // arbitration is never interrupted merely because a new command arrived.
    (void)abortMcpTxBounded();
    ++can_tx_errors_; noteCanTxFailure(now); return false;
  }
  (void)mcpBitModify(REG_TXB0CTRL, 0x70U, 0x00U);

  const uint32_t id = frame->id & CANARD_CAN_EXT_ID_MASK;
  const uint8_t sidh = static_cast<uint8_t>(id >> 21U);
  const uint8_t sidl = static_cast<uint8_t>(((id >> 13U) & 0xE0U) |
      ((id >> 16U) & 0x03U) | 0x08U);
  const uint8_t eid8 = static_cast<uint8_t>(id >> 8U);
  const uint8_t eid0 = static_cast<uint8_t>(id);
  uint8_t tx[15]{};
  tx[0] = MCP_WRITE; tx[1] = REG_TXB0SIDH;
  tx[2] = sidh; tx[3] = sidl; tx[4] = eid8; tx[5] = eid0;
  tx[6] = frame->data_len & 0x0FU;
  std::memcpy(&tx[7], frame->data, frame->data_len);
  if (!mcpTransfer(tx, nullptr, static_cast<uint16_t>(7U + frame->data_len))) {
    ++can_tx_errors_; noteCanTxFailure(now); return false;
  }
  const uint8_t rts = MCP_RTS_TX0;
  if (!mcpTransfer(&rts, nullptr, 1U)) {
    ++can_tx_errors_; noteCanTxFailure(now); return false;
  }
  mcp_tx_pending_ = true;
  mcp_tx_started_cycles_ = DWT->CYCCNT;
  return true; // accepted by MCP mailbox; physical completion is asynchronous
}

void Neo3ProSensors::serviceMcpTx() {
  if (!mcp_tx_pending_) return;
  uint8_t st = 0U;
  if (!mcpRead(REG_TXB0CTRL, &st)) {
    ++can_tx_errors_;
    noteCanTxFailure(HAL_GetTick());
    mcp_tx_pending_ = false;
    // Do not destroy unrelated queued transfers because one mailbox status read
    // failed. ArduPilot retries/ages queued transfers independently.
    can_recovery_pending_ = true;
    return;
  }
  if ((st & TXREQ) == 0U) {
    mcp_tx_pending_ = false;
    mcp_tx_started_cycles_ = 0U;
    if ((st & 0x50U) == 0U) {
      ++can_tx_frames_;
      can_tx_fail_streak_ = 0U;
      (void)mcpBitModify(REG_TXB0CTRL, 0x70U, 0U);
      return;
    }
    ++can_tx_errors_;
    noteCanTxFailure(HAL_GetTick());
    // The failed mailbox frame was already handed to MCP and popped from the
    // libcanard queue. Preserve the remaining queue; each transfer carries its
    // own deadline and will age out naturally, matching ArduPilot semantics.
    (void)mcpBitModify(REG_TXB0CTRL, 0x70U, 0U);
    return;
  }

  const uint32_t cycles_per_us = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  const uint32_t elapsed_cycles = static_cast<uint32_t>(DWT->CYCCNT - mcp_tx_started_cycles_);
  if (elapsed_cycles >= cycles_per_us * CAN_TX_HW_DEADLINE_US) {
    // Physical mailbox deadline expired (peer vanished/no ACK). Abort once,
    // discard remaining tails of that transfer, and back off. No main-loop wait.
    (void)abortMcpTxBounded();
    ++can_tx_errors_;
    noteCanTxFailure(HAL_GetTick());
    // Timeout is local to this physical mailbox. Keep unrelated queued frames;
    // libcanard's per-frame deadline performs stale cleanup.
  }
}

bool Neo3ProSensors::flushCanTx(uint8_t frame_budget) {
  serviceMcpTx();
  if (mcp_tx_pending_ || !canTxPermitted(HAL_GetTick())) return true;
  const uint64_t now_us = Board_MonotonicMicros64();
  while (frame_budget-- > 0U) {
    CanardCANFrame *frame = canardPeekTxQueue(&canard_);
    if (!frame) return true;
#if CANARD_ENABLE_DEADLINE
    const uint64_t deadline = canardPeekTxQueueDeadline(&canard_);
    if (deadline != 0U && static_cast<int64_t>(now_us - deadline) >= 0) {
      ++can_tx_expired_count_;
      canardPopTxQueue(&canard_);
      continue;
    }
#endif
    if (!writeOneCanFrame(frame)) return false;
    // Once the MCP mailbox owns the frame, remove it from libcanard. Physical
    // completion is asynchronous and independently bounded by the MCP deadline.
    canardPopTxQueue(&canard_);
    return true;
  }
  return true;
}

bool Neo3ProSensors::broadcastDroneCan(uint64_t signature, uint16_t data_type_id,
                                        uint8_t *transfer_id, uint8_t priority,
                                        const void *payload, uint16_t payload_len) {
  if (!transfer_id || !payload || canardGetLocalNodeID(&canard_) == 0U) return false;
  const uint64_t deadline_us = Board_MonotonicMicros64() + CAN_TX_QUEUE_DEADLINE_US;
  const int16_t queued = canardBroadcast(&canard_, signature, data_type_id, transfer_id,
      priority, payload, payload_len, deadline_us);
  if (queued <= 0) {
    ++can_tx_errors_;
    return false;
  }
  (void)flushCanTx(1U);
  return true;
}

bool Neo3ProSensors::requestService(uint8_t destination_node_id, uint64_t signature,
                                      uint8_t service_id, uint8_t *transfer_id,
                                      const void *payload, uint16_t payload_len) {
  if (destination_node_id == 0U || destination_node_id > 125U || !transfer_id ||
      (payload_len > 0U && payload == nullptr) || canardGetLocalNodeID(&canard_) == 0U)
    return false;
  // All host-originated DroneCAN service traffic in this driver targets a node
  // already discovered by NodeStatus. Never queue a request to an electrically
  // absent/stale peer: an unacknowledged MCP2515 TX would raise TEC and can mask
  // the actual peer-loss diagnosis.
  const uint32_t now_ms = HAL_GetTick();
  if (!dnaNodeSeen(destination_node_id) ||
      static_cast<uint32_t>(now_ms - dna_last_seen_ms_[destination_node_id]) > CAN_NODE_STALE_MS)
    return false;
  const uint64_t deadline_us = Board_MonotonicMicros64() + CAN_TX_QUEUE_DEADLINE_US;
  const int16_t queued = canardRequestOrRespond(&canard_, destination_node_id, signature,
      service_id, transfer_id, CANARD_TRANSFER_PRIORITY_LOW, CanardRequest, payload, payload_len, deadline_us);
  if (queued <= 0) { ++can_tx_errors_; return false; }
  (void)flushCanTx(1U);
  return true;
}

bool Neo3ProSensors::requestGetNodeInfo(uint8_t node_id) {
  last_identity_request_ms_ = HAL_GetTick();
  return requestService(node_id, SIG_GET_NODE_INFO, SID_GET_NODE_INFO,
                        &get_node_info_transfer_id_, nullptr, 0U);
}

bool Neo3ProSensors::requestParamRead(uint8_t node_id, const char *name, uint8_t pending_id) {
  if (!name || pending_id == 0U || params_.pending != 0U) return false;
  const size_t n = std::strlen(name);
  if (n == 0U || n > 92U) return false;
  // uavcan.protocol.param.GetSet request with classic-CAN TAO:
  // index:uint13=0 + Value union tag EMPTY:uint3=0 -> exactly two zero bytes,
  // followed directly by the parameter name (tail array; no length field).
  uint8_t payload[94]{};
  std::memcpy(&payload[2], name, n);
  params_.pending = pending_id;
  params_.request_ms = HAL_GetTick();
  const bool ok = requestService(node_id, SIG_PARAM_GETSET, SID_PARAM_GETSET,
                                 &param_getset_transfer_id_, payload,
                                 static_cast<uint16_t>(2U + n));
  // Keep pending set even on a transient TX failure; serviceDiscovery() clears
  // it after the normal timeout, preventing high-rate retry/flood loops.
  return ok;
}

bool Neo3ProSensors::requestParamSetInt(uint8_t node_id, const char *name, int64_t value, uint8_t pending_id) {
  if (!name || pending_id == 0U || params_.pending != 0U) return false;
  const size_t n = std::strlen(name);
  if (n == 0U || n > 92U) return false;
  // GetSet request: index:uint13=0, Value union tag INTEGER=1 at bit13,
  // int64 value at bit16, followed by TAO parameter name.
  uint8_t payload[102]{};
  uint8_t tag = 1U;
  canardEncodeScalar(payload, 13U, 3U, &tag);
  canardEncodeScalar(payload, 16U, 64U, &value);
  std::memcpy(&payload[10], name, n);
  params_.pending = pending_id;
  params_.request_ms = HAL_GetTick();
  return requestService(node_id, SIG_PARAM_GETSET, SID_PARAM_GETSET,
                        &param_getset_transfer_id_, payload, static_cast<uint16_t>(10U + n));
}

void Neo3ProSensors::serviceDiscovery(uint32_t now_ms) {
  if (!canTxPermitted(now_ms)) return;

  // Verify a newly observed NodeStatus source in parallel with the old primary,
  // as ArduPilot's DNA server does. The candidate gains authority only after
  // GetNodeInfo proves CUAV_GPS board identity.
  if (discovery_node_id_ != 0U && discovery_node_id_ != primary_node_id_) {
    // Discovery is driven by a *fresh* NodeStatus observation. ArduPilot's DNA
    // server sends an immediate verification when a heartbeat is seen, then its
    // independent 5-second verifier handles registered nodes. Do not keep a
    // stale candidate in a fast GetNodeInfo retry loop after the peer vanished.
    const uint8_t candidate = discovery_node_id_;
    const bool candidate_fresh = dnaNodeSeen(candidate) &&
        static_cast<uint32_t>(now_ms - dna_last_seen_ms_[candidate]) <= CAN_NODE_STALE_MS;
    if (!candidate_fresh) {
      discovery_node_id_ = 0U;
      last_identity_request_ms_ = 0U;
      telemetry_dirty_ |= DIRTY_PEER_HEALTH;
      return;
    }
    if (static_cast<uint32_t>(now_ms - last_identity_request_ms_) >= NODE_INFO_RETRY_MS)
      (void)requestGetNodeInfo(candidate);
    return;
  }

  if (identity_.verified_cuav_gps && dnaNodeVerified(primary_node_id_) &&
      dna_server_state_ != DnaServerState::DUPLICATE_NODES) {
    // Do not generate parameter traffic toward a peer whose NodeStatus is stale
    // or unhealthy. This is a health/failsafe condition, not a CAN-bus fault.
    if (!dnaNodeSeen(primary_node_id_) || !dnaNodeHealthy(primary_node_id_) ||
        static_cast<uint32_t>(now_ms - dna_last_seen_ms_[primary_node_id_]) > CAN_NODE_STALE_MS) return;
    if (params_.pending != 0U && static_cast<uint32_t>(now_ms - params_.request_ms) > 1500U)
      params_.pending = 0U;
    if (params_.pending == 0U) {
      if (!params_.can_node_valid)
        (void)requestParamRead(primary_node_id_, "CAN_NODE", 1U);
      else if (!params_.can_baudrate_valid)
        (void)requestParamRead(primary_node_id_, "CAN_BAUDRATE", 2U);
      else if (!params_.led_brightness_valid)
        (void)requestParamRead(primary_node_id_, "LED_BRIGHTNESS", 3U);
      else if (params_.led_brightness != NEO3PRO_LED_BRIGHTNESS_TARGET) {
        params_.led_brightness_valid = false;
        (void)requestParamSetInt(primary_node_id_, "LED_BRIGHTNESS",
                                 NEO3PRO_LED_BRIGHTNESS_TARGET, 3U);
      } else if (!params_.baro_enable_valid)
        (void)requestParamRead(primary_node_id_, "BARO_ENABLE", 4U);
    }
    return;
  }
  if (discovery_node_id_ != 0U && dnaNodeSeen(discovery_node_id_) &&
      static_cast<uint32_t>(now_ms - dna_last_seen_ms_[discovery_node_id_]) <= CAN_NODE_STALE_MS &&
      static_cast<uint32_t>(now_ms - last_identity_request_ms_) >= NODE_INFO_RETRY_MS)
    (void)requestGetNodeInfo(discovery_node_id_);
}

bool Neo3ProSensors::sendLedRgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t light_id) {
  // Match ArduPilot DroneCAN_RGB_LED exactly: light_id=0 and RGB888 reduced
  // to RGB565 (R>>3, G>>2, B>>3). Brightness belongs to AP_Periph's
  // LED_BRIGHTNESS parameter; do not alter requested color/brightness here.
  const uint32_t now = HAL_GetTick();
  if (!sensorAuthorityValid(primary_node_id_, now) ||
      !params_.led_brightness_valid ||
      params_.led_brightness != NEO3PRO_LED_BRIGHTNESS_TARGET) return false;
  // uavcan.equipment.indication.LightsCommand uses TAO on classic CAN. One
  // SingleLightCommand is exactly: light_id (8) + RGB565 (5+6+5) = 3 bytes.
  const uint16_t rgb565 = static_cast<uint16_t>((red >> 3U) & 0x1FU) |
      static_cast<uint16_t>(((green >> 2U) & 0x3FU) << 5U) |
      static_cast<uint16_t>(((blue >> 3U) & 0x1FU) << 11U);
  const uint8_t payload[3] = {light_id, static_cast<uint8_t>(rgb565),
                              static_cast<uint8_t>(rgb565 >> 8U)};
  return broadcastDroneCan(SIG_LIGHTS_COMMAND, DTID_LIGHTS_COMMAND, &lights_transfer_id_,
                           CANARD_TRANSFER_PRIORITY_LOW, payload, sizeof(payload));
}

bool Neo3ProSensors::drainCan(uint8_t frame_budget, bool allow_recovery,
                                 uint32_t time_budget_us) {
  if (!can_ok_ || frame_budget == 0U || time_budget_us == 0U) return can_ok_;
  const uint32_t started_cycles = DWT->CYCCNT;
  const uint32_t cycles_per_us = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  uint8_t frames = 0U;
  auto elapsedUs = [&]() -> uint32_t {
    return static_cast<uint32_t>(DWT->CYCCNT - started_cycles) / cycles_per_us;
  };
  auto finish = [&](bool ok) -> bool {
    const uint32_t elapsed = elapsedUs();
    if (elapsed > can_rx_service_max_us_) can_rx_service_max_us_ = elapsed;
    return ok;
  };

  // MCP2515 INT is active-low and remains asserted while an enabled condition
  // is pending. Drain by interrupt state and a hard time budget instead of a
  // fixed loop count alone, mirroring ArduPilot's "process until no data" CAN
  // servicing while preserving cooperative latency for USB/HMI/watchdog work.
  while (frames < frame_budget && elapsedUs() < time_budget_us) {
    uint8_t rx_status = 0U;
    if (!mcpReadStatus(&rx_status)) {
      if (allow_recovery) can_recovery_pending_ = true;
      return finish(false);
    }
    uint8_t flags = static_cast<uint8_t>(rx_status & (RX0IF | RX1IF));
    if (flags == 0U && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
      // INT can remain low for ERRIF even when neither RX buffer is full.
      uint8_t canintf = 0U;
      if (!mcpRead(REG_CANINTF, &canintf)) {
        if (allow_recovery) can_recovery_pending_ = true;
        return finish(false);
      }
      flags |= static_cast<uint8_t>(canintf & ERRIF);
    }

    if (flags & ERRIF) {
      uint8_t eflg = 0U;
      if (mcpRead(REG_EFLG, &eflg)) {
        bool overflow = false;
        if (eflg & RX0OVR) { ++can_rx0_overflows_; overflow = true; }
        if (eflg & RX1OVR) { ++can_rx1_overflows_; overflow = true; }
        if (overflow) ++can_overflows_;
        if (eflg & (RX0OVR | RX1OVR))
          (void)mcpBitModify(REG_EFLG, RX0OVR | RX1OVR, 0U);
      }
      (void)mcpBitModify(REG_CANINTF, ERRIF, 0U);
    }

    bool consumed = false;
    if ((flags & RX0IF) && frames < frame_budget) {
      if (!readOneCanFrame(0U)) {
        if (allow_recovery) can_recovery_pending_ = true;
        return finish(false);
      }
      // MCP2515 READ RX BUFFER clears RX0IF automatically when CS rises.
      ++frames;
      consumed = true;
    }
    if ((flags & RX1IF) && frames < frame_budget && elapsedUs() < time_budget_us) {
      if (!readOneCanFrame(1U)) {
        if (allow_recovery) can_recovery_pending_ = true;
        return finish(false);
      }
      // MCP2515 READ RX BUFFER clears RX1IF automatically when CS rises.
      ++frames;
      consumed = true;
    }

    if (!consumed) {
      // Once INT is released, there is no enabled MCP event pending. If INT is
      // still low, loop once more because ERRIF may have just been cleared or a
      // new RX frame may have arrived during the SPI transaction.
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) != GPIO_PIN_RESET) {
        Board_McpIntClear();
        return finish(true);
      }
    }
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET)
    ++can_rx_budget_exhaustions_;
  else
    Board_McpIntClear();
  return finish(true);
}

void Neo3ProSensors::recoverCan() {
  const uint32_t now = HAL_GetTick();
  if (last_can_recovery_ms_ != 0U &&
      static_cast<uint32_t>(now - last_can_recovery_ms_) < CAN_RECOVERY_COOLDOWN_MS)
    return;
  last_can_recovery_ms_ = now;
  ++recoveries_;
  can_recovery_pending_ = false;
  clearCanardTxQueue();
  mcp_tx_pending_ = false;
  mcp_tx_started_cycles_ = 0U;
  params_.pending = 0U;
  (void)abortMcpTxBounded();

  // A hard CAN controller reset breaks frame continuity. Purge the software RX
  // queue and libcanard RX states exactly at this boundary. Transfer-ID counters
  // for our outgoing DTO/services live outside CanardInstance and are preserved.
  raw_can_head_ = raw_can_tail_ = raw_can_count_ = 0U;
  canardInit(&canard_, canard_pool_, sizeof(canard_pool_), &Neo3ProSensors::onTransfer,
             &Neo3ProSensors::shouldAccept, this);
  canardSetLocalNodeID(&canard_, DRONECAN_HOST_NODE_ID);
  oscillator_locked_ = true;
  if (!Board_ReinitSpi2()) {
    can_ok_ = false;
    can_recovery_pending_ = true;
  } else {
    can_ok_ = initMcp(8U);
    can_recovery_pending_ = !can_ok_;
  }
  oscillator_locked_ = true;
  can_tx_backoff_until_ms_ = now + CAN_TX_BACKOFF_BASE_MS;
}

void Neo3ProSensors::pollRealtime() {
  if (!can_ok_) return;
  if (!Board_McpIntPending()) return;
  // Dedicated SPI2 means this bounded drain can run safely during TFT SPI1 work.
  (void)drainCan(MAX_RX_REALTIME, false, CAN_RX_REALTIME_BUDGET_US);
}

void Neo3ProSensors::serviceTelemetry(uint32_t now_ms) {
  if (telemetry_dirty_ & DIRTY_DNA) {
    char body[96];
    std::snprintf(body, sizeof(body), "%u,%u,%u",
                  static_cast<unsigned>(DRONECAN_HOST_NODE_ID),
                  static_cast<unsigned>(allocated_node_id_),
                  static_cast<unsigned>(dna_published_uid_len_));
    writeV2Record("SENS:DNA:", body);
    telemetry_dirty_ &= ~DIRTY_DNA;
  }
  if (telemetry_dirty_ & DIRTY_DNA_SERVER) {
    uint16_t seen_count = 0U, verified_count = 0U, healthy_count = 0U;
    for (uint8_t id = 1U; id <= DroneCanDnaDatabase::kMaxNodeId; ++id) {
      if (dna_seen_[id]) ++seen_count;
      if (dna_verified_[id]) ++verified_count;
      if (dna_healthy_[id]) ++healthy_count;
    }
    char body[300];
    std::snprintf(body, sizeof(body),
        "%lu,%lu,%d,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%u,%u",
        static_cast<unsigned long>(++dna_server_sequence_),
        static_cast<unsigned long>(now_ms), static_cast<int>(dna_server_state_),
        static_cast<unsigned>(dna_fault_node_id_), static_cast<unsigned>(seen_count),
        static_cast<unsigned>(verified_count), static_cast<unsigned>(healthy_count),
        static_cast<unsigned>(dna_db_.registeredCount()),
        static_cast<unsigned long>(dna_db_.usedJournalSlots()),
        static_cast<unsigned long>(dna_db_.totalJournalSlots()),
        static_cast<unsigned long>(dna_db_.journalWrites()),
        static_cast<unsigned long>(dna_db_.invalidJournalRecords()),
        dna_db_.storageOk() ? 1U : 0U, dna_db_.storageFull() ? 1U : 0U,
        static_cast<unsigned long>(dna_duplicate_count_),
        static_cast<unsigned long>(dna_verification_failures_),
        static_cast<unsigned long>(dna_allocation_failures_),
        static_cast<unsigned long>(dna_storage_faults_),
        static_cast<unsigned>(dna_curr_verifying_node_), dna_nodeinfo_response_received_ ? 1U : 0U);
    writeV2Record("SENS:DNASRV:", body);
    telemetry_dirty_ &= ~DIRTY_DNA_SERVER;
  }
  if (telemetry_dirty_ & DIRTY_PEER_HEALTH) {
    const uint32_t node_age = node_.seen ? static_cast<uint32_t>(now_ms - node_.received_ms) : 0xFFFFFFFFUL;
    const uint32_t raw_age = last_raw_frame_ms_ != 0U ? static_cast<uint32_t>(now_ms - last_raw_frame_ms_) : 0xFFFFFFFFUL;
    char body[220];
    std::snprintf(body, sizeof(body), "%lu,%u,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu",
        static_cast<unsigned long>(now_ms), static_cast<unsigned>(peer_state_),
        static_cast<unsigned long>(peer_state_since_ms_), static_cast<unsigned>(primary_node_id_),
        static_cast<unsigned>(discovery_node_id_), static_cast<unsigned>(node_.health),
        static_cast<unsigned>(node_.mode), static_cast<unsigned long>(node_age),
        static_cast<unsigned long>(raw_age), static_cast<unsigned long>(peer_restart_count_),
        static_cast<unsigned long>(peer_lost_count_), static_cast<unsigned long>(node_unhealthy_count_));
    writeV2Record("SENS:NEOHEALTH:", body);
    telemetry_dirty_ &= ~DIRTY_PEER_HEALTH;
  }
  if (telemetry_dirty_ & DIRTY_NODEINFO) {
    publishNodeIdentity(); telemetry_dirty_ &= ~DIRTY_NODEINFO;
  }
  if (telemetry_dirty_ & DIRTY_PARAM) {
    publishParamProbe(last_param_name_, last_param_value_); telemetry_dirty_ &= ~DIRTY_PARAM;
  }
  if (telemetry_dirty_ & DIRTY_BUTTON) {
    publishButton(safetyPressed()); telemetry_dirty_ &= ~DIRTY_BUTTON;
  }
  if ((telemetry_dirty_ & DIRTY_NODE) && identity_.verified_cuav_gps &&
      node_.source_node_id == primary_node_id_ &&
      now_ms - last_node_usb_ms_ >= USB_NODE_PERIOD_MS) {
    publishNodeStatus(); telemetry_dirty_ &= ~DIRTY_NODE; last_node_usb_ms_ = now_ms;
  }
  if ((telemetry_dirty_ & DIRTY_GNSSSTAT) && now_ms - last_gnssstat_usb_ms_ >= USB_GNSSSTAT_PERIOD_MS) {
    publishGnssStatus(); telemetry_dirty_ &= ~DIRTY_GNSSSTAT; last_gnssstat_usb_ms_ = now_ms;
  }
  if (telemetry_dirty_ & DIRTY_GNSS_FIX) {
    publishGnssPro(); publishGnssCov(); if (ecef_.valid) publishEcef(); publishGnss();
    telemetry_dirty_ &= ~(DIRTY_GNSS_FIX | DIRTY_GNSS_META | DIRTY_ECEF);
    last_gnssmeta_usb_ms_ = now_ms;
  } else if ((telemetry_dirty_ & DIRTY_GNSS_META) && now_ms - last_gnssmeta_usb_ms_ >= USB_GNSS_META_PERIOD_MS) {
    publishGnssPro(); telemetry_dirty_ &= ~DIRTY_GNSS_META; last_gnssmeta_usb_ms_ = now_ms;
  }
  if ((telemetry_dirty_ & DIRTY_HEADING) && now_ms - last_heading_usb_ms_ >= USB_HEADING_PERIOD_MS) {
    publishHeading(); telemetry_dirty_ &= ~DIRTY_HEADING; last_heading_usb_ms_ = now_ms;
  }
  if ((telemetry_dirty_ & DIRTY_MAG) && now_ms - last_mag_usb_ms_ >= USB_MAG_PERIOD_MS) {
    publishMagPro(); publishMag(); telemetry_dirty_ &= ~DIRTY_MAG; last_mag_usb_ms_ = now_ms;
  }
  if ((telemetry_dirty_ & DIRTY_BARO) && now_ms - last_baro_usb_ms_ >= USB_BARO_PERIOD_MS) {
    publishBaroPressure(); telemetry_dirty_ &= ~DIRTY_BARO; last_baro_usb_ms_ = now_ms;
  }
  if ((telemetry_dirty_ & DIRTY_TEMP) && now_ms - last_temp_usb_ms_ >= USB_TEMP_PERIOD_MS) {
    publishBaroTemperature(); telemetry_dirty_ &= ~DIRTY_TEMP; last_temp_usb_ms_ = now_ms;
  }
}

bool Neo3ProSensors::nodeOperationalFresh(uint32_t now_ms) const {
  return identity_.verified_cuav_gps && primary_node_id_ != 0U &&
         dna_db_.storageOk() && dnaNodeSeen(primary_node_id_) &&
         dnaNodeVerified(primary_node_id_) && dnaNodeHealthy(primary_node_id_) &&
         dna_server_state_ != DnaServerState::DUPLICATE_NODES &&
         static_cast<uint32_t>(now_ms - dna_last_seen_ms_[primary_node_id_]) <= CAN_NODE_STALE_MS;
}

bool Neo3ProSensors::sensorAuthorityValid(uint8_t source_node_id, uint32_t now_ms) const {
  return source_node_id != 0U && source_node_id == primary_node_id_ &&
         nodeOperationalFresh(now_ms);
}

void Neo3ProSensors::updatePeerFailsafe(uint32_t now_ms) {
  PeerState next = PeerState::NO_PEER;
  if (!can_ok_) {
    next = PeerState::BUS_FAULT;
  } else if (!dna_db_.storageOk()) {
    next = PeerState::DNA_STORAGE_FAULT;
  } else if (dna_server_state_ == DnaServerState::DUPLICATE_NODES) {
    next = PeerState::DUPLICATE_NODE;
  } else if (primary_node_id_ == 0U || !identity_.verified_cuav_gps ||
             !dnaNodeVerified(primary_node_id_)) {
    next = (discovery_node_id_ != 0U ||
            (last_raw_frame_ms_ != 0U &&
             static_cast<uint32_t>(now_ms - last_raw_frame_ms_) <= CAN_NODE_STALE_MS))
               ? PeerState::DISCOVERING : PeerState::NO_PEER;
  } else if (!dnaNodeSeen(primary_node_id_)) {
    next = (discovery_node_id_ != 0U && discovery_node_id_ != primary_node_id_)
               ? PeerState::DISCOVERING : PeerState::VERIFIED_WAIT_STATUS;
  } else if (static_cast<uint32_t>(now_ms - dna_last_seen_ms_[primary_node_id_]) > CAN_NODE_STALE_MS) {
    next = PeerState::PEER_LOST;
  } else if (!dnaNodeHealthy(primary_node_id_)) {
    // Same condition used by ArduPilot DNA server: health must be OK and mode
    // must be OPERATIONAL. This revokes sensor authority, not the CAN driver.
    next = PeerState::NODE_UNHEALTHY;
  } else {
    next = PeerState::ACTIVE;
  }

  const bool entering_peer_lost = next == PeerState::PEER_LOST &&
                                  peer_state_ != PeerState::PEER_LOST;
  const bool leaving_peer_lost = next != PeerState::PEER_LOST &&
                                 peer_state_ == PeerState::PEER_LOST;

  if (entering_peer_lost) {
    ++peer_lost_count_;
    peer_loss_recovery_attempted_ = false;
    // Fail quiet when the verified peer disappears. Any pending service frame
    // can no longer receive an ACK and would otherwise keep increasing TEC.
    // Purge/abort once on entry; CAN RX stays enabled for immediate peer return.
    if (mcp_tx_pending_) (void)abortMcpTxBounded();
    clearCanardTxQueue();
    ++can_peer_loss_tx_purge_count_;
    can_tx_backoff_until_ms_ = std::max<uint32_t>(can_tx_backoff_until_ms_,
        now_ms + CAN_RECOVERY_COOLDOWN_MS);
  }

  // Raw CAN can become silent slightly *after* NodeStatus crosses the stale
  // threshold. The old transition-only check missed that case forever. While
  // PEER_LOST persists, wait until all raw traffic is stale, then reinitialize
  // MCP/SPI exactly once for this loss episode. This recovers an RX-stuck MCP2515
  // without creating a reset loop when the NEO3 is physically unplugged.
  if (next == PeerState::PEER_LOST && !peer_loss_recovery_attempted_) {
    const bool raw_silent = last_raw_frame_ms_ != 0U &&
        static_cast<uint32_t>(now_ms - last_raw_frame_ms_) > CAN_NODE_STALE_MS;
    const bool cooldown_ready = last_can_recovery_ms_ == 0U ||
        static_cast<uint32_t>(now_ms - last_can_recovery_ms_) >= CAN_RECOVERY_COOLDOWN_MS;
    if (raw_silent && cooldown_ready) {
      peer_loss_recovery_attempted_ = true;
      ++can_silent_recovery_count_;
      recoverCan();
    }
  }
  if (leaving_peer_lost) peer_loss_recovery_attempted_ = false;

  if (next != peer_state_) {
    if (next == PeerState::NODE_UNHEALTHY) ++node_unhealthy_count_;
    peer_state_ = next;
    peer_state_since_ms_ = now_ms;
    telemetry_dirty_ |= DIRTY_PEER_HEALTH;
  }
}

void Neo3ProSensors::pollSafetyIo() {
  // Momentary DroneCAN safety button state is freshness-based; no local GPIO.
}

bool Neo3ProSensors::safetyPressed() const {
  return button_.seen && button_.button == 1U &&
         static_cast<uint32_t>(HAL_GetTick() - button_.received_ms) <= BUTTON_STALE_MS;
}

void Neo3ProSensors::poll() {
  uint32_t now_ms = HAL_GetTick();
  serviceCanHealth(now_ms);
  // ArduPilot does not erase verified identity merely because NodeStatus is
  // temporarily stale. Freshness changes health/authority; transport identity is
  // retained and can recover immediately if the same node returns. A different
  // source is verified in parallel before it can replace the primary.
  updatePeerFailsafe(now_ms);
  if (!can_ok_) {
    if (static_cast<uint32_t>(now_ms - last_poll_ms_) >= 500U) {
      last_poll_ms_ = now_ms;
      can_recovery_pending_ = true;
    }
    serviceTelemetry(now_ms);
    publishHardware(false);
    return;
  }
  const bool int_active = Board_McpIntPending();
  if (int_active || static_cast<uint32_t>(now_ms - last_poll_ms_) >= 10U) {
    last_poll_ms_ = now_ms;
    (void)drainCan(MAX_RX_PER_POLL, true, CAN_RX_MAIN_BUDGET_US);
  }
  // Protocol/DSDL work happens after the tiny MCP hardware buffers are empty,
  // matching ArduPilot/PX4 CANIface receive-queue architecture.
  processRawCanQueue(MAX_RX_PER_POLL, CAN_RX_DECODE_BUDGET_US);
  // RX callbacks stamp NodeStatus/GetNodeInfo with the current HAL tick. Refresh
  // the poll clock before freshness arithmetic; otherwise a callback crossing a
  // millisecond boundary can make (old_now - new_received_ms) wrap to UINT32_MAX
  // and create a 1 ms false PEER_LOST transition.
  now_ms = HAL_GetTick();
  updatePeerFailsafe(now_ms);
  // TX is drained cooperatively after RX. Multi-frame DNA responses can span
  // several loop iterations while TXB0 is busy without blocking sensor RX.
  flushCanTx(4U);
  if (static_cast<uint32_t>(now_ms - last_cleanup_ms_) >= 1000U) {
    last_cleanup_ms_ = now_ms;
    canardCleanupStaleTransfers(&canard_, Board_MonotonicMicros64());
  }
  if (button_.published_pressed && !safetyPressed()) {
    button_.published_pressed = false;
    telemetry_dirty_ |= DIRTY_BUTTON;
  }
  maybeProbeOscillator(now_ms);
  serviceDiscovery(now_ms);
  serviceDnaVerification(now_ms);
  serviceTelemetry(now_ms);
  publishHardware(false);
}

bool Neo3ProSensors::acceptSource(uint8_t source_node_id) const {
  return primary_node_id_ == 0U || source_node_id == primary_node_id_;
}

bool Neo3ProSensors::shouldAccept(const CanardInstance *ins, uint64_t *signature,
                                  uint16_t id, CanardTransferType type, uint8_t source_node_id) {
  if (!signature) return false;
  const auto *self = ins ? static_cast<const Neo3ProSensors *>(canardGetUserReference(ins)) : nullptr;

  if (type == CanardTransferTypeResponse) {
    if (source_node_id == 0U || source_node_id > DroneCanDnaDatabase::kMaxNodeId) return false;
    if (id == SID_GET_NODE_INFO) {
      if (self != nullptr && source_node_id != self->primary_node_id_ &&
          source_node_id != self->discovery_node_id_ &&
          source_node_id != self->dna_curr_verifying_node_) return false;
      *signature = SIG_GET_NODE_INFO;
      return true;
    }
    if (id == SID_PARAM_GETSET) {
      if (self != nullptr && source_node_id != self->primary_node_id_) return false;
      *signature = SIG_PARAM_GETSET;
      return true;
    }
    return false;
  }

  if (type != CanardTransferTypeBroadcast) return false;
  // Dynamic allocation request is the only anonymous transfer consumed here.
  if (id == DTID_ALLOCATION && source_node_id == 0U) {
    *signature = SIG_ALLOCATION;
    return true;
  }
  if (source_node_id == 0U) return false;

  // Do not source-filter at the libcanard subscription boundary. ArduPilot
  // keeps transport listeners active for all matching DTOs and applies node
  // selection/authority above the transport layer.

  // ArduPilot subscribes transport listeners independently from node identity
  // verification.  Do the same here: accept known sensor DTOs into libcanard so
  // transfer-ID/toggle state remains synchronized during discovery. onTransfer()
  // quarantines their decoded authority until GetNodeInfo verifies CUAV_GPS.
  switch (id) {
    case DTID_NODE_STATUS: *signature = SIG_NODE_STATUS; return true;
    case DTID_MAG1: *signature = SIG_MAG1; return true;
    case DTID_MAG2: *signature = SIG_MAG2; return true;
    case DTID_MAG_HIRES: *signature = SIG_MAG_HIRES; return true;
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
  self->oscillator_locked_ = true;

  if (transfer->transfer_type == CanardTransferTypeResponse) {
    if (transfer->data_type_id == SID_GET_NODE_INFO) self->decodeGetNodeInfoResponse(transfer);
    else if (transfer->data_type_id == SID_PARAM_GETSET) self->decodeParamGetSetResponse(transfer);
    return;
  }

  if (transfer->data_type_id == DTID_ALLOCATION && transfer->source_node_id == 0U) {
    self->decodeAllocation(transfer);
    return;
  }

  if (transfer->data_type_id == DTID_NODE_STATUS && transfer->source_node_id != 0U) {
    // ArduPilot DNA tracks NodeStatus for every node, not only the selected
    // sensor. decodeNodeStatus() updates per-node seen/healthy masks while only
    // the verified primary updates authoritative sensor state.
    self->decodeNodeStatus(transfer);
    if (transfer->source_node_id != self->primary_node_id_) return;
  }

  // Until identity is verified, sensor DTOs are transport-only quarantine.
  if (self->primary_node_id_ == 0U) return;

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
    case DTID_MAG_HIRES: self->decodeMagHiRes(transfer); break;
    case DTID_PRESSURE: self->decodeStaticPressure(transfer); break;
    case DTID_TEMPERATURE: self->decodeStaticTemperature(transfer); break;
    case DTID_NODE_STATUS: self->decodeNodeStatus(transfer); break;
    case DTID_GNSS_STATUS: self->decodeGnssStatus(transfer); break;
    case DTID_GNSS_HEADING: self->decodeHeading(transfer); break;
    case DTID_BUTTON: self->decodeButton(transfer); break;
    default: break;
  }
}

void Neo3ProSensors::decodeAllocation(const CanardRxTransfer *t) {
  if (!t || t->source_node_id != 0U || t->payload_len < 2U || t->payload_len > 7U) {
    ++can_decode_errors_;
    return;
  }

  uint8_t requested = 0U;
  bool first_part = false;
  // canardDecodeScalar() uses UAVCAN DSDL bit ordering here. For the wire byte
  // (node_id << 1) | first_part, scalar offsets 0..6 decode node_id and offset
  // 7 decodes first_part_of_unique_id.
  if (!decodeScalar(t, 0U, 7U, false, &requested) ||
      !decodeScalar(t, 7U, 1U, false, &first_part)) {
    ++can_decode_errors_;
    return;
  }
  (void)requested; // ArduPilot DNA deliberately ignores preferred node ID.
  const uint8_t uid_len = static_cast<uint8_t>(t->payload_len - 1U);
  uint8_t uid_part[6]{};
  for (uint8_t i = 0U; i < uid_len; ++i) {
    if (!decodeScalar(t, 8U + static_cast<uint32_t>(i) * 8U, 8U, false, &uid_part[i])) {
      ++can_decode_errors_;
      return;
    }
  }

  const uint32_t now = HAL_GetTick();
  // UAVCAN allocation follow-up timeout and first-part restart semantics match
  // AP_DroneCAN_DNA_Server::handle_allocation().
  if (static_cast<uint32_t>(now - dna_last_request_ms_) > 500U) dna_unique_id_len_ = 0U;
  if (first_part) dna_unique_id_len_ = 0U;
  else if (dna_unique_id_len_ == 0U) return;
  dna_last_request_ms_ = now;

  if (static_cast<uint16_t>(dna_unique_id_len_) + uid_len > sizeof(dna_unique_id_)) {
    dna_unique_id_len_ = 0U;
    ++can_decode_errors_;
    return;
  }
  std::memcpy(&dna_unique_id_[dna_unique_id_len_], uid_part, uid_len);
  dna_unique_id_len_ = static_cast<uint8_t>(dna_unique_id_len_ + uid_len);

  uint8_t response[17]{};
  uint8_t response_node = 0U;
  const bool complete = dna_unique_id_len_ == sizeof(dna_unique_id_);
  if (complete) {
    response_node = dna_db_.handleAllocation(dna_unique_id_);
    if (response_node == 0U) {
      ++dna_allocation_failures_;
      if (!dna_db_.storageOk() || dna_db_.storageFull()) ++dna_storage_faults_;
      telemetry_dirty_ |= DIRTY_DNA_SERVER;
      dna_unique_id_len_ = 0U;
      // Match ArduPilot: do not broadcast a completed allocation with node_id=0;
      // let the allocatee time out and retry instead.
      return;
    }
    allocated_node_id_ = response_node;
  }
  // Allocator responses are non-anonymous: first_part_of_unique_id must be 0.
  // DSDL wire layout is [node_id:bits1..7 | first_part:bit0].
  response[0] = static_cast<uint8_t>((response_node & 0x7FU) << 1U);
  std::memcpy(&response[1], dna_unique_id_, dna_unique_id_len_);
  (void)broadcastDroneCan(SIG_ALLOCATION, DTID_ALLOCATION, &dna_transfer_id_,
                          CANARD_TRANSFER_PRIORITY_LOW, response,
                          static_cast<uint16_t>(1U + dna_unique_id_len_));

  if (complete) {
    dna_published_uid_len_ = dna_unique_id_len_;
    telemetry_dirty_ |= DIRTY_DNA | DIRTY_DNA_SERVER;
    dna_unique_id_len_ = 0U;
  }
}

void Neo3ProSensors::decodeGetNodeInfoResponse(const CanardRxTransfer *t) {
  if (!t || t->transfer_type != CanardTransferTypeResponse || t->source_node_id == 0U ||
      t->payload_len < 41U) { ++can_decode_errors_; return; }

  NodeIdentityState next{};
  next.seen = true;
  next.source_node_id = t->source_node_id;
  next.received_ms = HAL_GetTick();
  uint8_t sw_flags = 0U, cert_len = 0U;
  if (!decodeScalar(t, 56U, 8U, false, &next.sw_major) ||
      !decodeScalar(t, 64U, 8U, false, &next.sw_minor) ||
      !decodeScalar(t, 72U, 8U, false, &sw_flags) ||
      !decodeScalar(t, 80U, 32U, false, &next.vcs_commit) ||
      !decodeScalar(t, 112U, 64U, false, &next.image_crc) ||
      !decodeScalar(t, 176U, 8U, false, &next.hw_major) ||
      !decodeScalar(t, 184U, 8U, false, &next.hw_minor)) {
    ++can_decode_errors_; return;
  }
  (void)sw_flags;
  for (uint8_t i = 0U; i < 16U; ++i) {
    if (!decodeScalar(t, 192U + static_cast<uint32_t>(i) * 8U, 8U, false, &next.unique_id[i])) {
      ++can_decode_errors_; return;
    }
  }
  if (!decodeScalar(t, 320U, 8U, false, &cert_len)) { ++can_decode_errors_; return; }
  const uint32_t name_ofs = 328U + static_cast<uint32_t>(cert_len) * 8U;
  const uint32_t total_bits = static_cast<uint32_t>(t->payload_len) * 8U;
  if (name_ofs > total_bits) { ++can_decode_errors_; return; }
  const uint32_t name_len = (total_bits - name_ofs) / 8U;
  if (name_len == 0U || name_len > 80U) { ++can_decode_errors_; return; }
  for (uint32_t i = 0U; i < name_len; ++i) {
    uint8_t ch = 0U;
    if (!decodeScalar(t, name_ofs + i * 8U, 8U, false, &ch)) { ++can_decode_errors_; return; }
    next.name[i] = static_cast<char>(ch);
  }
  next.name[name_len] = '\0';
  next.board_id = static_cast<uint16_t>((static_cast<uint16_t>(next.hw_major) << 8U) | next.hw_minor);
  next.verified_cuav_gps = next.board_id == CUAV_GPS_APJ_BOARD_ID &&
                           std::strncmp(next.name, "org.ardupilot.", 14U) == 0;

  const uint8_t source = next.source_node_id;
  if (source == 0U || source > DroneCanDnaDatabase::kMaxNodeId) {
    ++can_decode_errors_;
    return;
  }
  bool duplicate = false;
  bool storage_fault = false;
  if (source <= DroneCanDnaDatabase::kMaxNodeId) {
    if (dna_db_.isRegistered(source)) {
      duplicate = !dna_db_.uidMatchesNode(source, next.unique_id, sizeof(next.unique_id));
    } else {
      duplicate = dna_db_.handleNodeInfo(source, next.unique_id);
      if (duplicate && (!dna_db_.storageOk() || dna_db_.storageFull())) {
        storage_fault = true;
        duplicate = false;
      }
    }
  } else {
    duplicate = true;
  }

  if (storage_fault) {
    dna_verified_[source] = false;
    ++dna_storage_faults_;
    telemetry_dirty_ |= DIRTY_DNA_SERVER | DIRTY_PEER_HEALTH;
  } else if (duplicate) {
    // Exact ArduPilot failsafe semantics: a Node ID associated with another UID
    // is a DUPLICATE_NODES server fault. Never grant sensor authority to it.
    dna_verified_[source] = false;
    dna_server_state_ = DnaServerState::DUPLICATE_NODES;
    dna_fault_node_id_ = source;
    std::memset(dna_fault_node_name_, 0, sizeof(dna_fault_node_name_));
    std::strncpy(dna_fault_node_name_, next.name, sizeof(dna_fault_node_name_) - 1U);
    ++dna_duplicate_count_;
    telemetry_dirty_ |= DIRTY_DNA_SERVER | DIRTY_PEER_HEALTH | DIRTY_NODEINFO;
  } else {
    // register_uid() may move this UID from an old node ID to this one. Clear
    // verification for any database registration that no longer exists.
    for (uint8_t id = 1U; id <= DroneCanDnaDatabase::kMaxNodeId; ++id)
      if (dna_verified_[id] && !dna_db_.isRegistered(id)) dna_verified_[id] = false;
    dna_verified_[source] = true;
    if (source == dna_curr_verifying_node_) dna_nodeinfo_response_received_ = true;
    recomputeDnaServerState();
  }

  // Publish CUAV identity only when its node-ID/UID association is verified.
  // Other DroneCAN nodes remain in the DNA database but cannot replace NEO.
  if (next.verified_cuav_gps && dna_verified_[source] &&
      (primary_node_id_ == 0U || source == primary_node_id_ || source == discovery_node_id_)) {
    identity_ = next;
    telemetry_dirty_ |= DIRTY_NODEINFO;
  } else if (source == primary_node_id_ && !dna_verified_[source]) {
    telemetry_dirty_ |= DIRTY_NODEINFO;
  }

  // GetNodeInfoResponse embeds NodeStatus as the first 56 bits. ArduPilot updates
  // node health from this response too, so do it for every verified response.
  NodeState response_status{};
  response_status.seen = true;
  response_status.source_node_id = source;
  response_status.received_ms = HAL_GetTick();
  if (!decodeScalar(t, 0U, 32U, false, &response_status.uptime_sec) ||
      !decodeScalar(t, 32U, 2U, false, &response_status.health) ||
      !decodeScalar(t, 34U, 3U, false, &response_status.mode) ||
      !decodeScalar(t, 37U, 3U, false, &response_status.sub_mode) ||
      !decodeScalar(t, 40U, 16U, false, &response_status.vendor_status)) {
    ++can_decode_errors_; return;
  }
  // Do not use the embedded status uptime for reboot detection. A multi-frame
  // GetNodeInfo response can complete after a newer 1 Hz NodeStatus broadcast,
  // so its embedded uptime may legitimately be one tick older. ArduPilot uses
  // the backwards-uptime condition only as logging lifecycle information here;
  // authoritative reboot detection below is performed on NodeStatus broadcasts.
  const uint32_t prior_uptime = dna_last_uptime_[source];
  markDnaNodeStatus(response_status);
  if (prior_uptime > response_status.uptime_sec) dna_last_uptime_[source] = prior_uptime;

  if (next.verified_cuav_gps && dna_verified_[source] && !duplicate && !storage_fault) {
    const bool primary_missing = primary_node_id_ == 0U;
    const bool current_stale = primary_node_id_ != 0U &&
        (!dnaNodeSeen(primary_node_id_) ||
         static_cast<uint32_t>(HAL_GetTick() - dna_last_seen_ms_[primary_node_id_]) > CAN_NODE_STALE_MS);
    if (source == primary_node_id_ || primary_missing || current_stale) {
      const bool authority_switch = primary_node_id_ != source;
      primary_node_id_ = source;
      discovery_node_id_ = source;
      node_ = response_status;
      telemetry_dirty_ |= DIRTY_NODE | DIRTY_PEER_HEALTH;
      if (authority_switch) params_ = ParamProbeState{};
    }
  } else if (discovery_node_id_ == source && source != primary_node_id_ && !next.verified_cuav_gps) {
    discovery_node_id_ = 0U;
  }

}

void Neo3ProSensors::decodeParamGetSetResponse(const CanardRxTransfer *t) {
  if (!t || t->transfer_type != CanardTransferTypeResponse || params_.pending == 0U) return;
  uint8_t tag = 0U;
  if (!decodeScalar(t, 5U, 3U, false, &tag)) { ++can_decode_errors_; params_.pending = 0U; return; }
  if (tag != 1U) { // AP_Int8/AP_Int16/AP_Int32 are represented by integer_value.
    ++can_decode_errors_; params_.pending = 0U; return;
  }
  int64_t value = 0;
  if (!decodeScalar(t, 8U, 64U, true, &value)) { ++can_decode_errors_; params_.pending = 0U; return; }
  const uint8_t which = params_.pending;
  params_.pending = 0U;
  const char *param_name = nullptr;
  if (which == 1U) { params_.can_node = value; params_.can_node_valid = true; param_name = "CAN_NODE"; }
  else if (which == 2U) { params_.can_baudrate = value; params_.can_baudrate_valid = true; param_name = "CAN_BAUDRATE"; }
  else if (which == 3U) { params_.led_brightness = value; params_.led_brightness_valid = true; param_name = "LED_BRIGHTNESS"; }
  else if (which == 4U) { params_.baro_enable = value; params_.baro_enable_valid = true; param_name = "BARO_ENABLE"; }
  else if (which == 5U) { params_.gps1_type = value; params_.gps1_type_valid = true; param_name = "GPS1_TYPE"; }
  else if (which == 6U) { params_.compass_enable = value; params_.compass_enable_valid = true; param_name = "COMPASS_ENABLE"; }
  if (param_name != nullptr) {
    std::snprintf(last_param_name_, sizeof(last_param_name_), "%s", param_name);
    last_param_value_ = value;
    telemetry_dirty_ |= DIRTY_PARAM;
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
  // DroneCAN/AP_Periph may encode an unavailable DOP as 0.  DOP is strictly
  // positive, so never let a zero/negative placeholder overwrite a previously
  // valid value.  ROS still receives the raw Auxiliary fields separately.
  if (std::isfinite(pdop) && pdop > 0.0F) gnss_.pdop = pdop;

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

  telemetry_dirty_ |= DIRTY_GNSS_FIX | DIRTY_GNSS_META;
  if (ecef_.valid) telemetry_dirty_ |= DIRTY_ECEF;
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
  // AP_Periph currently fills HDOP/VDOP but can leave PDOP/GDOP/etc at zero.
  // Treat non-positive DOP as 'not supplied' instead of 'perfect geometry'.
  const auto keep_dop = [](float incoming, float &dst) {
    if (std::isfinite(incoming) && incoming > 0.0F) dst = incoming;
  };
  keep_dop(gdop, gnss_.gdop); keep_dop(pdop, gnss_.pdop);
  keep_dop(hdop, gnss_.hdop); keep_dop(vdop, gnss_.vdop);
  keep_dop(tdop, gnss_.tdop); keep_dop(ndop, gnss_.ndop); keep_dop(edop, gnss_.edop);
  gnss_.sats_visible = visible;
  if (used > 0U) gnss_.sats = used;
  if (gnss_.seen) telemetry_dirty_ |= DIRTY_GNSS_META;
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
  telemetry_dirty_ |= DIRTY_MAG;
}

void Neo3ProSensors::decodeMagHiRes(const CanardRxTransfer *t) {
  if (!t || t->payload_len < 13U) { ++can_decode_errors_; return; }
  uint8_t sid = 0U;
  float x = NAN, y = NAN, z = NAN;
  if (!decodeScalar(t, 0U, 8U, false, &sid) ||
      !decodeScalar(t, 8U, 32U, true, &x) ||
      !decodeScalar(t, 40U, 32U, true, &y) ||
      !decodeScalar(t, 72U, 32U, true, &z) ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    ++can_decode_errors_; return;
  }
  mag_.seen = true;
  mag_.source_node_id = t->source_node_id;
  mag_.received_ms = HAL_GetTick();
  mag_.sensor_id = sid;
  mag_.message_type = 3U; // dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes 1043
  mag_.x_ga = x; mag_.y_ga = y; mag_.z_ga = z;
  mag_.covariance_len = 0U;
  std::fill_n(mag_.covariance, 9U, NAN);
  telemetry_dirty_ |= DIRTY_MAG;
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
  telemetry_dirty_ |= DIRTY_BARO;
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
  telemetry_dirty_ |= DIRTY_TEMP;
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
  const uint8_t source = next.source_node_id;
  if (source == 0U || source > DroneCanDnaDatabase::kMaxNodeId) return;
  if (dna_last_uptime_[source] > next.uptime_sec && dna_last_uptime_[source] != 0U) {
    // Same behavior used by ArduPilot: uptime going backwards means a node
    // reboot, not a CAN bus fault. Force its UID verification to run again.
    ++peer_restart_count_;
    dna_verified_[source] = false;
    if (source == primary_node_id_) last_identity_request_ms_ = 0U;
  }
  markDnaNodeStatus(next);
  const bool primary_stale = primary_node_id_ == 0U || !dnaNodeSeen(primary_node_id_) ||
      static_cast<uint32_t>(next.received_ms - dna_last_seen_ms_[primary_node_id_]) > CAN_NODE_STALE_MS;
  if (source == primary_node_id_ || (primary_stale && source == discovery_node_id_)) {
    node_ = next;
    telemetry_dirty_ |= DIRTY_NODE | DIRTY_PEER_HEALTH;
  }
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
  telemetry_dirty_ |= DIRTY_GNSSSTAT;
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
  telemetry_dirty_ |= DIRTY_HEADING;
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
    telemetry_dirty_ |= DIRTY_BUTTON;
  }
}

void Neo3ProSensors::writeUsbLine(const char *line) {
  if (!line) return;
  const size_t n = strnlen(line, 620U);
  if (n == 0U || n >= 620U || !gUsb.writeLine(line)) ++sensor_usb_drops_;
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
  const float gs2 = gnss_.vel_n * gnss_.vel_n + gnss_.vel_e * gnss_.vel_e;
  const float gs = std::sqrt(gs2);
  float course = std::atan2(gnss_.vel_e, gnss_.vel_n) * 57.2957795F;
  if (course < 0.0F) course += 360.0F;
  // Fix2 does not contain a COG-accuracy field. Derive it from horizontal
  // velocity covariance instead of incorrectly substituting GNSS Heading
  // accuracy (absolute/moving-baseline heading is a different observable).
  float course_acc_deg = 180.0F;
  if (gnss_.vel_cov_valid && gs2 > 0.0025F) { // speed > 5 cm/s
    const float var_n = gnss_.covariance[3];
    const float var_e = gnss_.covariance[4];
    const float course_var = (gnss_.vel_n * gnss_.vel_n * var_e +
                              gnss_.vel_e * gnss_.vel_e * var_n) / (gs2 * gs2);
    if (std::isfinite(course_var) && course_var >= 0.0F)
      course_acc_deg = std::min(180.0F, std::sqrt(course_var) * 57.2957795F);
  }
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
      finiteOr(gnss_.sacc_mps, 5.0F), course_acc_deg,
      finiteOr(gnss_.pdop, 99.0F), finiteOr(gnss_.rate_hz, 0.0F),
      static_cast<double>(gnss_.mode), static_cast<double>(gnss_.sub_mode));
  writeV2Record("SENS:GNSS:", body);
}

void Neo3ProSensors::publishGnssPro() {
  if (!gnss_.seen) return;
  char body[600], network_us[24], gnss_us[24];
  formatU64(network_us, sizeof(network_us), gnss_.timestamp_usec);
  formatU64(gnss_us, sizeof(gnss_us), gnss_.gnss_timestamp_usec);
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%s,%s,%u,%u,%u,%u,%u,%u,%u,%ld,%ld,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.3f,%u,%u",
      static_cast<unsigned long>(++gnss_pro_sequence_),
      static_cast<unsigned long>(gnss_.received_ms), static_cast<unsigned>(gnss_.source_node_id),
      network_us, gnss_us,
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
  char body[600], px[24], py[24], pz[24];
  formatI64(px, sizeof(px), ecef_.position_xyz_mm[0]);
  formatI64(py, sizeof(py), ecef_.position_xyz_mm[1]);
  formatI64(pz, sizeof(pz), ecef_.position_xyz_mm[2]);
  int n = std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%.7g,%.7g,%.7g,%s,%s,%s,%u",
      static_cast<unsigned long>(++ecef_sequence_), static_cast<unsigned long>(ecef_.received_ms),
      static_cast<unsigned>(ecef_.source_node_id), ecef_.velocity_xyz[0], ecef_.velocity_xyz[1],
      ecef_.velocity_xyz[2], px, py, pz, static_cast<unsigned>(ecef_.covariance_len));
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

void Neo3ProSensors::publishNodeIdentity() {
  char uid_hex[33]{};
  for (uint8_t i = 0U; i < 16U; ++i)
    std::snprintf(&uid_hex[i * 2U], 3U, "%02X", static_cast<unsigned>(identity_.unique_id[i]));
  char body[260], image_crc[24];
  formatU64(image_crc, sizeof(image_crc), identity_.image_crc);
  std::snprintf(body, sizeof(body), "%lu,%u,%u,%u,%u,%u,%lu,%s,%s,%s",
      static_cast<unsigned long>(identity_.received_ms),
      static_cast<unsigned>(identity_.source_node_id), identity_.verified_cuav_gps ? 1U : 0U,
      static_cast<unsigned>(identity_.board_id), static_cast<unsigned>(identity_.sw_major),
      static_cast<unsigned>(identity_.sw_minor), static_cast<unsigned long>(identity_.vcs_commit),
      image_crc, uid_hex, identity_.name);
  writeV2Record("SENS:NODEINFO:", body);
}

void Neo3ProSensors::publishParamProbe(const char *name, int64_t value) {
  char body[120], value_text[24];
  formatI64(value_text, sizeof(value_text), value);
  std::snprintf(body, sizeof(body), "%lu,%u,%s,%s",
      static_cast<unsigned long>(HAL_GetTick()), static_cast<unsigned>(primary_node_id_),
      name ? name : "", value_text);
  writeV2Record("SENS:PARAM:", body);
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
  const uint32_t now = HAL_GetTick();
  return mag_.seen && sensorAuthorityValid(mag_.source_node_id, now) &&
         static_cast<uint32_t>(now - mag_.received_ms) <= MAG_STALE_MS;
}

uint32_t Neo3ProSensors::lastCanFrameAgeMs(uint32_t now_ms) const {
  return last_raw_frame_ms_ == 0U ? 0xFFFFFFFFUL : static_cast<uint32_t>(now_ms - last_raw_frame_ms_);
}

void Neo3ProSensors::publishHardware(bool force) {
  const uint32_t now = HAL_GetTick();
  if (!force && static_cast<uint32_t>(now - last_hw_publish_ms_) < HW_PUBLISH_MS) return;
  last_hw_publish_ms_ = now;
  const bool gnss_alive = gnss_.seen && sensorAuthorityValid(gnss_.source_node_id, now) &&
      static_cast<uint32_t>(now - gnss_.received_ms) <= GNSS_STALE_MS;
  // ArduPilot AP_GPS_DroneCAN assumes healthy until a GNSS Status report has
  // actually been seen; once present, respect its healthy flag.
  const bool gnss_health_ok = !gnss_status_.seen || gnss_status_.healthy;
  const bool gnss_ready = gnss_alive && gnss_health_ok && gnss_.status >= 2U;
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

  // Reuse the 10 Hz health snapshot. Diagnostics must not add three extra SPI2
  // transactions per publish cycle while RX/TX are active.
  const uint8_t tec = can_health_tec_;
  const uint8_t rec = can_health_rec_;
  const uint8_t eflg = can_health_eflg_;
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%lu,%lu",
      static_cast<unsigned long>(seq), static_cast<unsigned long>(now), can_ok_ ? 1U : 0U,
      static_cast<unsigned>(active_osc_mhz_), static_cast<unsigned>(primary_node_id_),
      static_cast<unsigned long>(raw_frames_), static_cast<unsigned long>(dronecan_transfers_),
      static_cast<unsigned long>(accepted_transfers_), static_cast<unsigned long>(foreign_node_drops_),
      static_cast<unsigned long>(spi_errors_), static_cast<unsigned long>(can_decode_errors_),
      static_cast<unsigned long>(can_overflows_), static_cast<unsigned long>(recoveries_),
      static_cast<unsigned long>(Board_SpiContentionCount()), static_cast<unsigned>(tec),
      static_cast<unsigned>(rec), static_cast<unsigned>(eflg),
      static_cast<unsigned long>(sensor_usb_drops_), static_cast<unsigned long>(gUsb.txDropped()));
  writeV2Record("SENS:HWPRO:", body);
  std::snprintf(body, sizeof(body), "%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%lu,%lu",
      static_cast<unsigned long>(raw_frames_),
      static_cast<unsigned long>(raw_allocation_frames_),
      static_cast<unsigned long>(raw_fix2_frames_),
      static_cast<unsigned long>(raw_node_frames_),
      static_cast<unsigned long>(raw_mag_frames_),
      static_cast<unsigned long>(raw_other_frames_),
      static_cast<unsigned>(active_osc_mhz_), oscillator_locked_ ? 1U : 0U,
      static_cast<unsigned>(DRONECAN_HOST_NODE_ID),
      static_cast<unsigned>(allocated_node_id_),
      static_cast<unsigned long>(can_tx_frames_),
      static_cast<unsigned long>(can_tx_errors_));
  writeV2Record("SENS:CANRAW:", body);
  std::snprintf(body, sizeof(body), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu",
      static_cast<unsigned long>(can_decode_errors_),
      static_cast<unsigned long>(can_rx_resync_events_),
      static_cast<unsigned long>(can_overflows_),
      static_cast<unsigned long>(can_rx0_overflows_),
      static_cast<unsigned long>(can_rx1_overflows_),
      static_cast<unsigned long>(can_rx_budget_exhaustions_),
      static_cast<unsigned long>(can_rx_service_max_us_),
      static_cast<unsigned long>(can_tx_abort_count_),
      static_cast<unsigned>(can_tx_fail_streak_),
      static_cast<unsigned long>(can_busoff_count_),
      static_cast<unsigned long>(can_tx_passive_probe_count_),
      static_cast<unsigned long>(static_cast<int32_t>(can_tx_backoff_until_ms_ - now) > 0
          ? can_tx_backoff_until_ms_ - now : 0U));
  writeV2Record("SENS:CANHEALTH:", body);
  const uint32_t raw_age = lastCanFrameAgeMs(now);
  const uint32_t node_age = (primary_node_id_ != 0U && dnaNodeSeen(primary_node_id_))
      ? static_cast<uint32_t>(now - dna_last_seen_ms_[primary_node_id_]) : 0xFFFFFFFFUL;
  std::snprintf(body, sizeof(body), "%lu,%u,%lu,%lu,%lu,%lu,%lu,%u,%u",
      static_cast<unsigned long>(seq), static_cast<unsigned>(peer_state_),
      static_cast<unsigned long>(raw_age), static_cast<unsigned long>(node_age),
      static_cast<unsigned long>(can_peer_loss_tx_purge_count_),
      static_cast<unsigned long>(can_silent_recovery_count_),
      static_cast<unsigned long>(dna_verification_failures_),
      mcp_tx_pending_ ? 1U : 0U, canTxPermitted(now) ? 1U : 0U);
  writeV2Record("SENS:CANGUARD:", body);
  const uint32_t rxseq = ++can_rx_diag_sequence_;
  std::snprintf(body, sizeof(body),
      "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
      static_cast<unsigned long>(rxseq),
      static_cast<unsigned long>(can_rx_not_wanted_),
      static_cast<unsigned long>(can_rx_wrong_address_),
      static_cast<unsigned long>(can_rx_missed_start_),
      static_cast<unsigned long>(can_rx_wrong_toggle_),
      static_cast<unsigned long>(can_rx_unexpected_tid_),
      static_cast<unsigned long>(can_rx_short_frame_),
      static_cast<unsigned long>(can_rx_bad_crc_),
      static_cast<unsigned long>(can_rx_oom_),
      static_cast<unsigned long>(can_rx_internal_),
      static_cast<unsigned long>(can_rx_incompatible_),
      static_cast<unsigned long>(can_tx_expired_count_),
      static_cast<unsigned long>(can_rx_budget_exhaustions_),
      static_cast<unsigned>(raw_can_count_), static_cast<unsigned>(raw_can_hwm_),
      static_cast<unsigned long>(raw_can_sw_overflows_),
      static_cast<unsigned long>(raw_can_processed_),
      static_cast<unsigned long>(Board_McpIntCount()),
      static_cast<unsigned long>(can_irq_fast_frames_),
      static_cast<unsigned long>(can_irq_spi_busy_deferrals_),
      static_cast<unsigned long>(can_irq_fast_budget_exhaustions_),
      static_cast<unsigned long>(can_irq_fast_max_us_));
  writeV2Record("SENS:CANRX:", body);
}

bool Neo3ProSensors::handleHostCommand(const char *command) {
  if (!command) return false;
  if (std::strcmp(command, "NEO:STATUS") == 0 || std::strcmp(command, "NEO:CAN:STATUS") == 0) {
    publishHardware(true); return true;
  }
  if (std::strcmp(command, "NEO:CAN:RECOVER") == 0) {
    recoverCan(); publishHardware(true); return true;
  }
  if (std::strcmp(command, "NEO:BARO:ON") == 0 || std::strcmp(command, "NEO:BARO:OFF") == 0) {
    if (!identity_.verified_cuav_gps || primary_node_id_ == 0U) {
      writeUsbLine("ERR:NEO:BARO:NODE"); return true;
    }
    params_.baro_enable_valid = false;
    const int64_t value = std::strcmp(command, "NEO:BARO:ON") == 0 ? 1 : 0;
    const bool ok = requestParamSetInt(primary_node_id_, "BARO_ENABLE", value, 4U);
    writeUsbLine(ok ? "ACK:NEO:BARO:REQUEST" : "ERR:NEO:BARO:TX");
    return true;
  }
  if (std::strncmp(command, "NEO:LED:BRIGHTNESS:", 19U) == 0) {
    unsigned value = 0U;
    if (std::sscanf(command + 19U, "%u", &value) != 1 || value > 100U) {
      writeUsbLine("ERR:NEO:LED:BRIGHTNESS:ARGS"); return true;
    }
    if (!identity_.verified_cuav_gps || primary_node_id_ == 0U || !node_.seen ||
        static_cast<uint32_t>(HAL_GetTick() - node_.received_ms) > 3000U) {
      writeUsbLine("ERR:NEO:LED:BRIGHTNESS:NODE"); return true;
    }
    params_.led_brightness_valid = false;
    const bool ok = requestParamSetInt(primary_node_id_, "LED_BRIGHTNESS",
                                       static_cast<int64_t>(value), 3U);
    writeUsbLine(ok ? "ACK:NEO:LED:BRIGHTNESS:REQUEST"
                    : "ERR:NEO:LED:BRIGHTNESS:TX");
    return true;
  }
  if (std::strcmp(command, "NEO:LED:OFF") == 0) {
    const bool ok = sendLedRgb(0U, 0U, 0U);
    writeUsbLine(ok ? "ACK:NEO:LED" : "ERR:NEO:LED:TX");
    return true;
  }
  if (std::strncmp(command, "NEO:LED:", 8U) == 0) {
    unsigned r = 0U, g = 0U, b = 0U;
    if (std::sscanf(command + 8U, "%u,%u,%u", &r, &g, &b) == 3 &&
        r <= 255U && g <= 255U && b <= 255U) {
      const bool ok = sendLedRgb(static_cast<uint8_t>(r), static_cast<uint8_t>(g),
                                 static_cast<uint8_t>(b));
      writeUsbLine(ok ? "ACK:NEO:LED" : "ERR:NEO:LED:TX");
      return true;
    }
    writeUsbLine("ERR:NEO:LED:ARGS");
    return true;
  }
  return false;
}

#endif
