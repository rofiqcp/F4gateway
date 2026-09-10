#pragma once

#include <cstddef>
#include <cstdint>

class UsbCdcPort {
 public:
  bool begin();
  void end();
  void poll();
  void service();
  void requestRecovery();
  int available() const;
  int read();
  int availableForWrite() const;
  int availableHighPriorityForWrite() const;
  std::size_t write(const uint8_t *data, std::size_t length);
  std::size_t writeHighPriority(const uint8_t *data, std::size_t length);
  bool writeLine(const char *line);
  bool writeLineHighPriority(const char *line);
  bool writeLineCritical(const char *line, uint32_t timeout_ms = 150U);
  void flush(uint32_t timeout_ms = 100U);
  bool connected() const;
  uint32_t rxDropped() const { return rx_dropped_; }
  uint32_t txDropped() const { return tx_dropped_; }

  void onReceive(const uint8_t *data, uint32_t length);
  void onTransmitComplete();
  void onUsbClassInit();
  void onUsbClassDeInit();
  uint32_t sessionGeneration() const { return usb_session_generation_; }
  uint32_t classInitCount() const { return usb_class_init_count_; }
  uint32_t classDeInitCount() const { return usb_class_deinit_count_; }
  uint32_t txAbortCount() const { return tx_abort_on_session_reset_; }
  uint32_t txCompleteCount() const { return tx_complete_count_; }
  uint32_t txStallRecoveryCount() const { return tx_stall_recovery_count_; }
  uint32_t softRestartCount() const { return usb_soft_restart_count_; }
  uint32_t rxPacketCount() const { return rx_packet_count_; }
  uint32_t txProgressStallCount() const { return tx_progress_stall_count_; }
  uint32_t lastRecoveryReason() const { return last_recovery_reason_; }
  uint32_t lastRepairAgeMs() const { return last_repair_age_ms_; }
  uint16_t lastRepairPending() const { return last_repair_pending_; }
  uint32_t lastRepairEpLength() const { return last_repair_ep_length_; }
  uint8_t lastRepairFlags() const { return last_repair_flags_; }
  bool txBusy() const { return tx_busy_; }
  uint32_t txBusyAgeMs() const;
  uint32_t lastTxCompleteAgeMs() const;
  uint32_t lastRxAgeMs() const;
  uint16_t rxQueueDepth() const;
  uint16_t txLowQueueDepth() const;
  uint16_t txHighQueueDepth() const;
  uint32_t cdcTxState() const;
#ifdef HMI_TEST_HOOKS
  void testSuppressTx(uint32_t duration_ms);
#endif
 private:
  static constexpr uint32_t kTxStallRepairMs = 250U;
  static constexpr uint16_t kRxSize = 4096U;
  static constexpr uint16_t kTxSize = 4096U;
  static constexpr uint16_t kHighTxSize = 4096U;
  uint8_t rx_[kRxSize]{};
  uint8_t tx_[kTxSize]{};
  uint8_t tx_high_[kHighTxSize]{};
  volatile uint16_t rx_head_{0U};
  volatile uint16_t rx_tail_{0U};
  volatile uint16_t tx_head_{0U};
  volatile uint16_t tx_tail_{0U};
  volatile uint16_t tx_high_head_{0U};
  volatile uint16_t tx_high_tail_{0U};
  volatile bool tx_busy_{false};
  volatile bool tx_active_high_{false};
  volatile bool tx_message_active_{false};
  volatile bool tx_message_high_{false};
  volatile bool tx_packet_ends_message_{false};
  volatile uint16_t tx_pending_{0U};
  volatile uint32_t rx_dropped_{0U};
  volatile uint32_t tx_dropped_{0U};
  volatile uint32_t tx_started_ms_{0U};
  volatile uint32_t tx_complete_ms_{0U};
  volatile uint32_t usb_session_generation_{0U};
  volatile uint32_t usb_class_init_count_{0U};
  volatile uint32_t usb_class_deinit_count_{0U};
  volatile uint32_t tx_abort_on_session_reset_{0U};
  volatile uint32_t tx_complete_count_{0U};
  volatile uint32_t tx_stall_recovery_count_{0U};
  volatile uint32_t usb_soft_restart_count_{0U};
  volatile uint32_t last_rx_ms_{0U};
  volatile uint32_t rx_packet_count_{0U};
  volatile uint32_t tx_progress_stall_count_{0U};
  volatile uint32_t last_recovery_reason_{0U}; // 0 none, 1 explicit, 2 wrapper/ST mismatch
  volatile uint32_t last_repair_age_ms_{0U};
  volatile uint16_t last_repair_pending_{0U};
  volatile uint32_t last_repair_ep_length_{0U};
  volatile uint8_t last_repair_flags_{0U}; // bit0 high, bit1 message-end
  volatile bool tx_stall_reported_{false};
  volatile bool recovery_pending_{false};
#ifdef HMI_TEST_HOOKS
  volatile uint32_t test_suppress_tx_until_ms_{0U};
#endif
  uint8_t tx_packet_[64]{};

  void resetSessionState(bool drop_queues, bool count_abort);
  bool startUsbStack();
  bool softRestartUsb();
};

extern UsbCdcPort gUsb;
