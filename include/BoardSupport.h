#pragma once

#include "stm32f4xx_hal.h"
#include <cstddef>
#include <cstdint>

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim11;

void Board_Init();
void Board_Service();
uint32_t Board_MaxServiceGapMs();
void Board_SetRealtimeServiceCallback(void (*callback)());
void Board_RealtimeService();
void Board_DelayUs(uint32_t microseconds);
bool Board_ReinitSpi1();
void Board_ReinitI2c1();
void Board_SetWatchdogCallback(void (*callback)());
void Board_WatchdogStart();
void Board_WatchdogStop();
void Board_BuzzerStart(uint16_t frequency_hz, uint16_t duration_ms);
void Board_BuzzerStop();

class HalUartPort {
 public:
  HalUartPort(UART_HandleTypeDef *handle, USART_TypeDef *instance)
      : handle_(handle), instance_(instance) {}
  bool begin(uint32_t baudrate);
  void end();
  int available() const;
  int read();
  int availableForWrite() const;
  std::size_t queuedForWrite() const;
  void discardPendingTx();
  void dropQueuedAfterActiveTx();
  std::size_t write(const uint8_t *data, std::size_t length);
  std::size_t write(uint8_t byte) { return write(&byte, 1U); }
  void flush();
  bool service();
  uint32_t overflowCount() const { return overflow_count_; }
  uint32_t errorCount() const { return error_count_; }
  uint32_t txDropped() const { return tx_dropped_; }
  uint32_t txSegmentsStarted() const { return tx_segments_started_; }
  uint32_t txSegmentsCompleted() const { return tx_segments_completed_; }
  uint32_t rxIrqBytes() const { return rx_irq_bytes_; }
  uint32_t lastRxIrqMs() const { return last_rx_irq_ms_; }
  uint32_t maxQueueBytes() const { return max_queue_bytes_; }
  void irqRxComplete();
  void irqTxComplete();
  void irqError();
 private:
  static constexpr uint16_t kRxSize = 2048U;
  static constexpr uint16_t kTxSize = 4096U;
  UART_HandleTypeDef *handle_;
  USART_TypeDef *instance_;
  volatile uint16_t rx_head_{0U};
  volatile uint16_t rx_tail_{0U};
  uint8_t rx_byte_{0U};
  uint8_t rx_buffer_[kRxSize]{};
  uint8_t tx_buffer_[kTxSize]{};
  volatile uint16_t tx_head_{0U};
  volatile uint16_t tx_tail_{0U};
  volatile uint16_t tx_pending_{0U};
  volatile bool tx_busy_{false};
  volatile uint32_t tx_dropped_{0U};
  volatile uint32_t overflow_count_{0U};
  volatile uint32_t error_count_{0U};
  volatile uint32_t tx_segments_started_{0U};
  volatile uint32_t tx_segments_completed_{0U};
  volatile uint32_t rx_irq_bytes_{0U};
  volatile uint32_t last_rx_irq_ms_{0U};
  volatile uint32_t max_queue_bytes_{0U};
  volatile bool rx_restart_required_{false};
};

extern HalUartPort gVescUart;
extern HalUartPort gGnssUart;
