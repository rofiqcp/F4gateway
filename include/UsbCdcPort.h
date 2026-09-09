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
  std::size_t write(const uint8_t *data, std::size_t length);
  bool writeLine(const char *line);
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
  uint8_t rx_[kRxSize]{};
  uint8_t tx_[kTxSize]{};
  volatile uint16_t rx_head_{0U};
  volatile uint16_t rx_tail_{0U};
  volatile uint16_t tx_head_{0U};
  volatile uint16_t tx_tail_{0U};
  volatile bool tx_busy_{false};
  volatile uint16_t tx_pending_{0U};
  volatile uint32_t rx_dropped_{0U};
  volatile uint32_t tx_dropped_{0U};
  uint8_t tx_packet_[64]{};
};

extern UsbCdcPort gUsb;
