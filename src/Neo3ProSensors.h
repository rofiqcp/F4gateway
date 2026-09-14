#pragma once

#ifdef NEO3PRO

#include <cstdint>
#include <cmath>
#include "canard.h"
#include "DroneCanDnaDatabase.h"

class Neo3ProSensors {
public:
  bool begin();
  void poll();
  // Called from the HMI realtime-yield path. MCP2515 owns dedicated SPI2; this
  // path only performs bounded RX draining and never probes/reconfigures the bus.
  void pollRealtime();
  // EXTI10 callback: bounded MCP2515 hardware-RX -> RAM FIFO evacuation only.
  // Never performs libcanard/DSDL/USB work.
  void irqFastDrain();
  void pollSafetyIo();
  bool safetyPressed() const;
  bool handleHostCommand(const char *command);

  bool gnssUartOk() const { return can_ok_; } // compatibility HMI accessor
  bool magOk() const;
  bool canOk() const { return can_ok_; }
  uint32_t magErrorCount() const { return can_decode_errors_; }
  uint32_t canSpiErrors() const { return spi_errors_; }
  uint32_t canRxFrames() const { return raw_frames_; }
  uint32_t canTransfers() const { return dronecan_transfers_; }
  uint32_t canDecodeErrors() const { return can_decode_errors_; }
  uint32_t canRecoveries() const { return recoveries_; }
  uint32_t canOverflows() const { return can_overflows_; }
  uint8_t canOscillatorMhz() const { return active_osc_mhz_; }
  uint32_t lastCanFrameAgeMs(uint32_t now_ms) const;
  uint8_t primaryNodeId() const { return primary_node_id_; }

private:
  struct GnssState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    uint32_t last_interval_ms{0U};
    float rate_hz{0.0F};
    uint64_t timestamp_usec{0U};
    uint64_t gnss_timestamp_usec{0U};
    uint8_t gnss_time_standard{0U};
    uint8_t num_leap_seconds{0U};
    int64_t lon_e8{0};
    int64_t lat_e8{0};
    int32_t height_ellipsoid_mm{0};
    int32_t height_msl_mm{0};
    float vel_n{0.0F}, vel_e{0.0F}, vel_d{0.0F};
    uint8_t covariance_len{0U};
    float covariance[36]{};
    bool pos_cov_valid{false};
    bool vel_cov_valid{false};
    float hacc_m{100.0F}, vacc_m{150.0F}, sacc_mps{5.0F};
    float gdop{NAN}, pdop{NAN}, hdop{NAN}, vdop{NAN};
    float tdop{NAN}, ndop{NAN}, edop{NAN};
    uint8_t sats_visible{0U};
    uint8_t sats{0U}, status{0U}, mode{0U}, sub_mode{0U};
  } gnss_{};

  struct EcefState {
    bool valid{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    float velocity_xyz[3]{};
    int64_t position_xyz_mm[3]{};
    uint8_t covariance_len{0U};
    float covariance[36]{};
  } ecef_{};

  struct MagState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    float x_ga{0.0F}, y_ga{0.0F}, z_ga{0.0F};
    uint8_t sensor_id{0U};
    uint8_t message_type{0U}; // 1=1001, 2=1002
    uint8_t covariance_len{0U};
    float covariance[9]{};
  } mag_{};

  struct BaroState {
    bool pressure_seen{false};
    bool temperature_seen{false};
    uint8_t source_node_id{0U};
    uint32_t pressure_ms{0U};
    uint32_t temperature_ms{0U};
    float pressure_pa{NAN};
    float pressure_variance{NAN};
    float temperature_k{NAN};
    float temperature_variance{NAN};
  } baro_{};

  struct NodeState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    uint32_t uptime_sec{0U};
    uint8_t health{0U};
    uint8_t mode{0U};
    uint8_t sub_mode{0U};
    uint16_t vendor_status{0U};
  } node_{};

  struct GnssStatusState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    uint32_t error_codes{0U};
    bool healthy{false};
    uint32_t status_bits{0U};
  } gnss_status_{};

  struct HeadingState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    bool valid{false};
    bool accuracy_valid{false};
    float heading_rad{NAN};
    float accuracy_rad{NAN};
  } heading_{};

  struct ButtonState {
    bool seen{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    uint8_t button{0U};
    uint8_t press_time{0U};
    bool published_pressed{false};
  } button_{};

  struct NodeIdentityState {
    bool seen{false};
    bool verified_cuav_gps{false};
    uint8_t source_node_id{0U};
    uint32_t received_ms{0U};
    uint8_t sw_major{0U}, sw_minor{0U};
    uint8_t hw_major{0U}, hw_minor{0U};
    uint16_t board_id{0U};
    uint32_t vcs_commit{0U};
    uint64_t image_crc{0U};
    uint8_t unique_id[16]{};
    char name[81]{};
  } identity_{};

  struct ParamProbeState {
    bool can_node_valid{false};
    bool can_baudrate_valid{false};
    bool led_brightness_valid{false};
    bool baro_enable_valid{false};
    bool gps1_type_valid{false};
    bool compass_enable_valid{false};
    int64_t can_node{0};
    int64_t can_baudrate{0};
    int64_t led_brightness{0};
    int64_t baro_enable{0};
    int64_t gps1_type{0};
    int64_t compass_enable{0};
    uint8_t pending{0U}; // 1 CAN_NODE, 2 CAN_BAUDRATE, 3 LED, 4 BARO, 5 GPS1_TYPE, 6 COMPASS_ENABLE
    uint32_t request_ms{0U};
  } params_{};

  struct RawCanSlot {
    uint32_t id{0U};
    uint64_t timestamp_us{0ULL};
    uint8_t dlc{0U};
    uint8_t data[8]{};
  };
  static constexpr uint16_t RAW_CAN_QUEUE_CAPACITY = 128U;

  enum class PeerState : uint8_t {
    NO_PEER = 0U,
    DISCOVERING = 1U,
    VERIFIED_WAIT_STATUS = 2U,
    ACTIVE = 3U,
    NODE_UNHEALTHY = 4U,
    PEER_LOST = 5U,
    BUS_FAULT = 6U,
    DUPLICATE_NODE = 7U,
    DNA_STORAGE_FAULT = 8U,
  };

  enum class DnaServerState : int8_t {
    NODE_STATUS_UNHEALTHY = -5,
    DUPLICATE_NODES = -2,
    HEALTHY = 0,
  };

  bool initMcp(uint8_t osc_mhz);
  bool mcpTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
  bool mcpFastTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t deadline_cycles);
  bool mcpFastReadOneCanFrame(uint8_t buffer_index, uint32_t deadline_cycles);
  bool parseAndQueueMcpRx(const uint8_t *rx, uint64_t timestamp_us);
  bool mcpWrite(uint8_t addr, uint8_t value);
  bool mcpRead(uint8_t addr, uint8_t *value);
  bool mcpReadStatus(uint8_t *status);
  bool mcpBitModify(uint8_t addr, uint8_t mask, uint8_t value);
  bool mcpSetMode(uint8_t mode);
  bool readOneCanFrame(uint8_t buffer_index);
  bool enqueueRawCan(uint32_t id, uint8_t dlc, const uint8_t *data, uint64_t timestamp_us);
  void processRawCanQueue(uint16_t frame_budget, uint32_t time_budget_us);
  void processRawCanFrame(const RawCanSlot &slot);
  void updatePeerFailsafe(uint32_t now_ms);
  bool nodeOperationalFresh(uint32_t now_ms) const;
  bool sensorAuthorityValid(uint8_t source_node_id, uint32_t now_ms) const;
  void buildOwnDnaUid(uint8_t out[16]) const;
  void markDnaNodeStatus(const NodeState &status);
  void serviceDnaVerification(uint32_t now_ms);
  void recomputeDnaServerState();
  bool dnaNodeSeen(uint8_t node_id) const;
  bool dnaNodeVerified(uint8_t node_id) const;
  bool dnaNodeHealthy(uint8_t node_id) const;
  bool writeOneCanFrame(const CanardCANFrame *frame);
  void serviceMcpTx();
  bool abortMcpTxBounded();
  void clearCanardTxQueue();
  bool canTxPermitted(uint32_t now_ms) const;
  void noteCanTxFailure(uint32_t now_ms);
  void serviceCanHealth(uint32_t now_ms);
  bool drainCan(uint8_t frame_budget, bool allow_recovery, uint32_t time_budget_us);
  bool flushCanTx(uint8_t frame_budget = 4U);
  bool broadcastDroneCan(uint64_t signature, uint16_t data_type_id, uint8_t *transfer_id,
                         uint8_t priority, const void *payload, uint16_t payload_len);
  bool requestService(uint8_t destination_node_id, uint64_t signature, uint8_t service_id,
                      uint8_t *transfer_id, const void *payload, uint16_t payload_len);
  bool requestGetNodeInfo(uint8_t node_id);
  bool requestParamRead(uint8_t node_id, const char *name, uint8_t pending_id);
  bool requestParamSetInt(uint8_t node_id, const char *name, int64_t value, uint8_t pending_id);
  void serviceDiscovery(uint32_t now_ms);
  void serviceTelemetry(uint32_t now_ms);
  bool sendLedRgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t light_id = 0U);
  void recoverCan();
  void maybeProbeOscillator(uint32_t now_ms);
  bool acceptSource(uint8_t source_node_id) const;

  static bool shouldAccept(const CanardInstance *ins, uint64_t *signature,
                           uint16_t data_type_id, CanardTransferType transfer_type,
                           uint8_t source_node_id);
  static void onTransfer(CanardInstance *ins, CanardRxTransfer *transfer);
  void decodeAllocation(const CanardRxTransfer *transfer);
  void decodeGetNodeInfoResponse(const CanardRxTransfer *transfer);
  void decodeParamGetSetResponse(const CanardRxTransfer *transfer);
  void decodeFix2(const CanardRxTransfer *transfer);
  void decodeAuxiliary(const CanardRxTransfer *transfer);
  void decodeMag(const CanardRxTransfer *transfer, bool v2);
  void decodeMagHiRes(const CanardRxTransfer *transfer);
  void decodeStaticPressure(const CanardRxTransfer *transfer);
  void decodeStaticTemperature(const CanardRxTransfer *transfer);
  void decodeNodeStatus(const CanardRxTransfer *transfer);
  void decodeGnssStatus(const CanardRxTransfer *transfer);
  void decodeHeading(const CanardRxTransfer *transfer);
  void decodeButton(const CanardRxTransfer *transfer);
  void decodeEcef(const CanardRxTransfer *transfer, uint32_t bit_offset);

  void publishGnss();
  void publishGnssPro();
  void publishGnssCov();
  void publishEcef();
  void publishMag();
  void publishMagPro();
  void publishBaroPressure();
  void publishBaroTemperature();
  void publishNodeStatus();
  void publishGnssStatus();
  void publishHeading();
  void publishButton(bool pressed);
  void publishNodeIdentity();
  void publishParamProbe(const char *name, int64_t value);
  void publishHardware(bool force = false);
  void writeUsbLine(const char *line);
  void writeV2Record(const char *prefix, const char *body);
  uint32_t gnssTowMs() const;

  CanardInstance canard_{};
  alignas(8) uint8_t canard_pool_[4096]{};
  bool can_ok_{false};
  volatile bool mcp_spi_busy_{false};
  bool oscillator_locked_{false};
  uint8_t active_osc_mhz_{0U};
  uint8_t probe_index_{0U};
  uint8_t primary_node_id_{0U};
  uint8_t discovery_node_id_{0U};
  uint32_t last_identity_request_ms_{0U};
  uint32_t profile_started_ms_{0U};
  uint32_t last_poll_ms_{0U};
  uint32_t last_cleanup_ms_{0U};
  uint32_t last_hw_publish_ms_{0U};
  uint32_t last_raw_frame_ms_{0U};
  uint32_t spi_errors_{0U};
  uint32_t raw_frames_{0U};
  uint32_t raw_allocation_frames_{0U};
  uint32_t raw_fix2_frames_{0U};
  uint32_t raw_node_frames_{0U};
  uint32_t raw_mag_frames_{0U};
  uint32_t raw_other_frames_{0U};
  uint32_t dronecan_transfers_{0U};
  uint32_t accepted_transfers_{0U};
  uint32_t foreign_node_drops_{0U};
  uint32_t can_decode_errors_{0U};
  uint32_t can_rx_resync_events_{0U};
  uint32_t can_rx_not_wanted_{0U};
  uint32_t can_rx_wrong_address_{0U};
  uint32_t can_rx_missed_start_{0U};
  uint32_t can_rx_wrong_toggle_{0U};
  uint32_t can_rx_unexpected_tid_{0U};
  uint32_t can_rx_short_frame_{0U};
  uint32_t can_rx_bad_crc_{0U};
  uint32_t can_rx_oom_{0U};
  uint32_t can_rx_internal_{0U};
  uint32_t can_rx_incompatible_{0U};
  uint32_t can_overflows_{0U};
  uint32_t can_rx0_overflows_{0U};
  uint32_t can_rx1_overflows_{0U};
  uint32_t can_rx_budget_exhaustions_{0U};
  uint32_t can_rx_service_max_us_{0U};
  RawCanSlot raw_can_queue_[RAW_CAN_QUEUE_CAPACITY]{};
  uint16_t raw_can_head_{0U};
  uint16_t raw_can_tail_{0U};
  uint16_t raw_can_count_{0U};
  uint16_t raw_can_hwm_{0U};
  uint32_t raw_can_sw_overflows_{0U};
  uint32_t raw_can_processed_{0U};
  uint32_t raw_can_enqueued_{0U};
  uint32_t raw_can_dequeued_{0U};
  uint32_t raw_can_nonextended_{0U};
  uint32_t raw_can_invalid_{0U};
  volatile uint32_t can_irq_fast_frames_{0U};
  volatile uint32_t can_irq_spi_busy_deferrals_{0U};
  volatile uint32_t can_irq_fast_budget_exhaustions_{0U};
  volatile uint32_t can_irq_fast_max_us_{0U};
  PeerState peer_state_{PeerState::NO_PEER};
  uint32_t peer_state_since_ms_{0U};
  uint32_t peer_restart_count_{0U};
  uint32_t peer_lost_count_{0U};
  uint32_t node_unhealthy_count_{0U};
  DroneCanDnaDatabase dna_db_{};
  bool dna_seen_[DroneCanDnaDatabase::kMaxNodeId + 1U]{};
  bool dna_verified_[DroneCanDnaDatabase::kMaxNodeId + 1U]{};
  bool dna_healthy_[DroneCanDnaDatabase::kMaxNodeId + 1U]{};
  uint32_t dna_last_seen_ms_[DroneCanDnaDatabase::kMaxNodeId + 1U]{};
  uint32_t dna_last_uptime_[DroneCanDnaDatabase::kMaxNodeId + 1U]{};
  DnaServerState dna_server_state_{DnaServerState::HEALTHY};
  uint8_t dna_fault_node_id_{0U};
  char dna_fault_node_name_[16]{};
  uint32_t dna_last_verification_request_ms_{0U};
  uint8_t dna_curr_verifying_node_{124U};
  bool dna_nodeinfo_response_received_{true};
  uint32_t dna_duplicate_count_{0U};
  uint32_t dna_verification_failures_{0U};
  uint32_t dna_allocation_failures_{0U};
  uint32_t dna_storage_faults_{0U};
  uint32_t dna_server_sequence_{0U};
  uint32_t recoveries_{0U};
  uint32_t can_tx_frames_{0U};
  uint32_t can_tx_errors_{0U};
  uint32_t can_tx_abort_count_{0U};
  uint32_t can_tx_expired_count_{0U};
  uint32_t can_busoff_count_{0U};
  uint32_t can_tx_passive_probe_count_{0U};
  uint32_t can_peer_loss_tx_purge_count_{0U};
  uint32_t can_silent_recovery_count_{0U};
  bool peer_loss_recovery_attempted_{false};
  uint32_t can_tx_backoff_until_ms_{0U};
  uint32_t last_can_health_ms_{0U};
  uint32_t last_can_recovery_ms_{0U};
  uint8_t can_health_tec_{0U};
  uint8_t can_health_rec_{0U};
  uint8_t can_health_eflg_{0U};
  uint8_t can_tx_fail_streak_{0U};
  bool can_recovery_pending_{false};
  bool mcp_tx_pending_{false};
  uint32_t mcp_tx_started_cycles_{0U};
  uint8_t dna_unique_id_[16]{};
  uint8_t dna_unique_id_len_{0U};
  uint8_t dna_published_uid_len_{0U};
  uint32_t dna_last_request_ms_{0U};
  uint8_t allocated_node_id_{0U};
  uint8_t dna_transfer_id_{0U};
  uint8_t lights_transfer_id_{0U};
  uint8_t get_node_info_transfer_id_{0U};
  uint8_t param_getset_transfer_id_{0U};
  uint32_t telemetry_dirty_{0U};
  uint32_t last_mag_usb_ms_{0U};
  uint32_t last_baro_usb_ms_{0U};
  uint32_t last_temp_usb_ms_{0U};
  uint32_t last_node_usb_ms_{0U};
  uint32_t last_gnssmeta_usb_ms_{0U};
  uint32_t last_gnssstat_usb_ms_{0U};
  uint32_t last_heading_usb_ms_{0U};
  char last_param_name_[24]{};
  int64_t last_param_value_{0};
  uint32_t gnss_sequence_{0U};
  uint32_t gnss_pro_sequence_{0U};
  uint32_t gnss_cov_sequence_{0U};
  uint32_t ecef_sequence_{0U};
  uint32_t mag_sequence_{0U};
  uint32_t mag_pro_sequence_{0U};
  uint32_t baro_pressure_sequence_{0U};
  uint32_t baro_temperature_sequence_{0U};
  uint32_t node_sequence_{0U};
  uint32_t gnss_status_sequence_{0U};
  uint32_t heading_sequence_{0U};
  uint32_t button_sequence_{0U};
  uint32_t hw_sequence_{0U};
  uint32_t can_rx_diag_sequence_{0U};
  uint32_t sensor_usb_drops_{0U};
};

#endif
