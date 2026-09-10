#pragma once

#include <cstddef>
#include <cstdint>

class UsbCdcPort {
 public:
  bool begin();
  void end();
  void poll();
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
 private:
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
  uint8_t tx_packet_[64]{};
};

extern UsbCdcPort gUsb;
