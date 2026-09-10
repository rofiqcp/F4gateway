#pragma once

#ifdef NEO3PRO

#include <cstdint>
#include <cmath>
#include "canard.h"

class Neo3ProSensors {
public:
  bool begin();
  void poll();
  // Called from the HMI realtime-yield path. It never probes/reconfigures the
  // MCP2515 and immediately returns while another SPI1 owner is active.
  void pollRealtime();
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

  bool initMcp(uint8_t osc_mhz);
  bool mcpTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
  bool mcpWrite(uint8_t addr, uint8_t value);
  bool mcpRead(uint8_t addr, uint8_t *value);
  bool mcpBitModify(uint8_t addr, uint8_t mask, uint8_t value);
  bool mcpSetMode(uint8_t mode);
  bool readOneCanFrame(uint8_t buffer_index);
  bool drainCan(uint8_t frame_budget, bool allow_recovery);
  void recoverCan();
  void maybeProbeOscillator(uint32_t now_ms);
  bool acceptSource(uint8_t source_node_id) const;

  static bool shouldAccept(const CanardInstance *ins, uint64_t *signature,
                           uint16_t data_type_id, CanardTransferType transfer_type,
                           uint8_t source_node_id);
  static void onTransfer(CanardInstance *ins, CanardRxTransfer *transfer);
  void decodeFix2(const CanardRxTransfer *transfer);
  void decodeAuxiliary(const CanardRxTransfer *transfer);
  void decodeMag(const CanardRxTransfer *transfer, bool v2);
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
  void publishHardware(bool force = false);
  void writeUsbLine(const char *line);
  void writeV2Record(const char *prefix, const char *body);
  uint32_t gnssTowMs() const;

  CanardInstance canard_{};
  alignas(8) uint8_t canard_pool_[4096]{};
  bool can_ok_{false};
  bool oscillator_locked_{false};
  uint8_t active_osc_mhz_{0U};
  uint8_t probe_index_{0U};
  uint8_t primary_node_id_{0U};
  uint32_t profile_started_ms_{0U};
  uint32_t last_poll_ms_{0U};
  uint32_t last_cleanup_ms_{0U};
  uint32_t last_hw_publish_ms_{0U};
  uint32_t last_raw_frame_ms_{0U};
  uint32_t spi_errors_{0U};
  uint32_t raw_frames_{0U};
  uint32_t dronecan_transfers_{0U};
  uint32_t accepted_transfers_{0U};
  uint32_t foreign_node_drops_{0U};
  uint32_t can_decode_errors_{0U};
  uint32_t can_overflows_{0U};
  uint32_t recoveries_{0U};
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
};

#endif
