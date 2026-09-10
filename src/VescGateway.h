#pragma once

#include "BoardSupport.h"
#include "UsbCdcPort.h"
#include <cstddef>
#include <cstdint>


class VescGateway {
 public:
  void begin();
  void poll();
  bool handleHostCommand(const char *command);
  bool maintenanceMode() const { return owner_ == Owner::MAINTENANCE; }
  void setSafetyStop(bool active);
  bool safetyStopActive() const { return safety_stop_active_; }
  bool uartOk() const { return uart_ok_; }
  uint32_t rxFrames() const { return rx_frames_; }
  uint32_t frameErrors() const { return rx_frame_errors_; }
  uint32_t recoveryCount() const { return uart_recovery_count_; }
  uint32_t usbDropFrames() const { return usb_drop_frames_; }
  uint32_t runtimeQueueDrops() const { return runtime_queue_drop_; }
  uint32_t activeBaud() const { return active_baud_; }
  uint32_t lastValidFrameAgeMs(uint32_t now) const {
    return ever_valid_frame_ ? static_cast<uint32_t>(now - last_valid_frame_ms_) : 0xFFFFFFFFUL;
  }

 private:
  enum class Owner : uint8_t { RUNTIME = 0, MAINTENANCE = 1 };
  static constexpr uint32_t kBaud = 115200;
  static constexpr size_t kChunkBytes = 512;
  static constexpr size_t kRxFrameBytes = 768;   // F103 VESC_MAX_FRAME <= 707 bytes
  static constexpr size_t kRxBufferBytes = 1536; // noise + at least one whole frame
  static constexpr uint32_t kRxFrameTimeoutMs = 150;
  static constexpr uint32_t kStatusPeriodMs = 1000;
  static constexpr uint32_t kRuntimeNoValidFrameRecoverMs = 1200;
  static constexpr size_t kRuntimeMaxQueuedBytes = 128U; // <12 ms at 115200 8N1
  static constexpr uint32_t kRuntimeRecoverCooldownMs = 1200;
  static constexpr uint8_t kCommMotorEstop = 159U;
  static constexpr uint16_t kSafetyRefreshHoldMs = 250U;
  static constexpr uint16_t kSafetyReleaseHoldMs = 600U;
  static constexpr uint32_t kSafetyRefreshPeriodMs = 50U;
  static constexpr uint32_t kMaintenanceLeaseMs = 5000U;
  static constexpr uint32_t kUartRetryMs = 1000U;

  Owner owner_{Owner::RUNTIME};
  uint8_t rx_chunk_[kRxBufferBytes]{};
  size_t rx_len_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t rx_bytes_{0};
  uint32_t tx_bytes_{0};
  uint32_t rejected_bytes_{0};
  uint32_t rx_frames_{0};
  uint32_t rx_frame_errors_{0};
  uint32_t rx_frame_prefix_errors_{0};
  uint32_t rx_frame_crc_errors_{0};
  uint32_t rx_frame_timeout_bytes_{0};
  uint32_t rx_frame_oversize_errors_{0};
  uint8_t rx_last_error_byte_{0};
  uint32_t last_valid_frame_error_count_{0};
  uint32_t usb_drop_frames_{0};
  uint32_t usb_p1_drop_frames_{0};
  uint32_t usb_p3_drop_frames_{0};
  uint32_t runtime_queue_drop_{0};
  uint32_t last_valid_frame_ms_{0};
  uint32_t last_runtime_tx_ms_{0};
  uint32_t runtime_tx_epoch_ms_{0};
  uint32_t last_recovery_ms_{0};
  uint32_t recovery_tx_marker_{0};
  uint32_t uart_recovery_count_{0};
  uint8_t recovery_streak_{0};
  bool ever_valid_frame_{false};
  bool safety_stop_active_{false};
  uint32_t last_safety_stop_ms_{0U};
  bool uart_ok_{false};
  uint32_t active_baud_{kBaud};
  uint32_t maintenance_activity_ms_{0U};
  uint32_t last_uart_retry_ms_{0U};

  static int hexNibble(char c);
  static uint16_t crc16(const uint8_t *data, size_t len);
  static const char *ownerName(Owner owner);
  bool forwardHex(const char *hex, Owner source);
  bool writeUsbBounded(const uint8_t *data, size_t len, uint32_t timeout_ms);
  void publishRxFrame(const uint8_t *data, size_t len);
  void serviceRxFrames();
  void publishStatus(bool force = false);
  void recoverRuntimeUart(uint32_t now);
  void recoveryTick(uint32_t now);
  bool sendSafetyStop(uint16_t hold_ms);
  void switchToRuntime(uint32_t now, bool announce);
};
