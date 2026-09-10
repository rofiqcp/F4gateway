#include "UsbCdcPort.h"

#include "stm32f4xx_hal.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"

#include <algorithm>
#include <cstring>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

UsbCdcPort gUsb;

namespace {
uint16_t RingUsed(uint16_t head, uint16_t tail, uint16_t size) {
  return head >= tail ? static_cast<uint16_t>(head - tail)
                      : static_cast<uint16_t>(size - tail + head);
}
uint16_t RingFree(uint16_t head, uint16_t tail, uint16_t size) {
  return static_cast<uint16_t>(size - RingUsed(head, tail, size) - 1U);
}
} // namespace

bool UsbCdcPort::begin() {
  rx_head_ = rx_tail_ = tx_head_ = tx_tail_ = 0U;
  tx_high_head_ = tx_high_tail_ = 0U;
  tx_busy_ = false;
  tx_active_high_ = false;
  tx_message_active_ = false;
  tx_message_high_ = false;
  tx_packet_ends_message_ = false;
  tx_pending_ = 0U;
  rx_dropped_ = tx_dropped_ = 0U;
  if (USBD_Init(&hUsbDeviceFS, &USBD_Desc, 0U) != USBD_OK)
    return false;
  if (USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS) != USBD_OK)
    return false;
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) !=
      USBD_OK)
    return false;
  return USBD_Start(&hUsbDeviceFS) == USBD_OK;
}

void UsbCdcPort::end() {
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
  tx_busy_ = false;
  tx_active_high_ = false;
  tx_message_active_ = false;
  tx_message_high_ = false;
  tx_packet_ends_message_ = false;
}

bool UsbCdcPort::connected() const {
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

int UsbCdcPort::available() const {
  return RingUsed(rx_head_, rx_tail_, kRxSize);
}

int UsbCdcPort::read() {
  if (rx_head_ == rx_tail_)
    return -1;
  const uint8_t value = rx_[rx_tail_];
  rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxSize);
  return value;
}

int UsbCdcPort::availableForWrite() const {
  return RingFree(tx_head_, tx_tail_, kTxSize);
}
int UsbCdcPort::availableHighPriorityForWrite() const {
  return RingFree(tx_high_head_, tx_high_tail_, kHighTxSize);
}

std::size_t UsbCdcPort::write(const uint8_t *data, std::size_t length) {
  if (data == nullptr || length == 0U || length >= kTxSize)
    return 0U;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint16_t free = RingFree(tx_head_, tx_tail_, kTxSize);
  if (free < length) {
    ++tx_dropped_;
    if (primask == 0U)
      __enable_irq();
    return 0U;
  }
  for (std::size_t i = 0; i < length; ++i) {
    tx_[tx_head_] = data[i];
    tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  }
  if (primask == 0U)
    __enable_irq();
  poll();
  return length;
}

std::size_t UsbCdcPort::writeHighPriority(const uint8_t *data,
                                          std::size_t length) {
  if (data == nullptr || length == 0U || length >= kHighTxSize)
    return 0U;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint16_t free = RingFree(tx_high_head_, tx_high_tail_, kHighTxSize);
  if (free < length) {
    ++tx_dropped_;
    if (primask == 0U)
      __enable_irq();
    return 0U;
  }
  for (std::size_t i = 0; i < length; ++i) {
    tx_high_[tx_high_head_] = data[i];
    tx_high_head_ = static_cast<uint16_t>((tx_high_head_ + 1U) % kHighTxSize);
  }
  if (primask == 0U)
    __enable_irq();
  poll();
  return length;
}

bool UsbCdcPort::writeLine(const char *line) {
  if (line == nullptr)
    return false;
  const std::size_t len = std::strlen(line);
  const std::size_t total = len + 2U;
  if (total >= kTxSize)
    return false;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (RingFree(tx_head_, tx_tail_, kTxSize) < total) {
    ++tx_dropped_;
    if (primask == 0U)
      __enable_irq();
    return false;
  }
  for (std::size_t i = 0U; i < len; ++i) {
    tx_[tx_head_] = static_cast<uint8_t>(line[i]);
    tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  }
  tx_[tx_head_] = '\r';
  tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  tx_[tx_head_] = '\n';
  tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  if (primask == 0U)
    __enable_irq();
  poll();
  return true;
}

bool UsbCdcPort::writeLineHighPriority(const char *line) {
  if (line == nullptr)
    return false;
  const std::size_t len = std::strlen(line);
  const std::size_t total = len + 2U;
  if (total >= kHighTxSize)
    return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (RingFree(tx_high_head_, tx_high_tail_, kHighTxSize) < total) {
    ++tx_dropped_;
    if (primask == 0U)
      __enable_irq();
    return false;
  }
  for (std::size_t i = 0U; i < len; ++i) {
    tx_high_[tx_high_head_] = static_cast<uint8_t>(line[i]);
    tx_high_head_ = static_cast<uint16_t>((tx_high_head_ + 1U) % kHighTxSize);
  }
  tx_high_[tx_high_head_] = '\r';
  tx_high_head_ = static_cast<uint16_t>((tx_high_head_ + 1U) % kHighTxSize);
  tx_high_[tx_high_head_] = '\n';
  tx_high_head_ = static_cast<uint16_t>((tx_high_head_ + 1U) % kHighTxSize);
  if (primask == 0U)
    __enable_irq();
  poll();
  return true;
}

bool UsbCdcPort::writeLineCritical(const char *line, uint32_t timeout_ms) {
  if (line == nullptr || !connected())
    return false;
  const uint32_t start = HAL_GetTick();
  do {
    if (writeLineHighPriority(line))
      return true;
    poll();
    HAL_Delay(1U);
  } while (static_cast<uint32_t>(HAL_GetTick() - start) < timeout_ms &&
           connected());
  return false;
}

void UsbCdcPort::poll() {
  if (!connected() || tx_busy_)
    return;
  const bool high_ready = tx_high_head_ != tx_high_tail_;
  const bool low_ready = tx_head_ != tx_tail_;
  if (!high_ready && !low_ready)
    return;

  // Priority is chosen only between complete newline-delimited messages. Once a
  // low-priority line has started, finish that line before allowing VESC P1 to
  // take the endpoint. This prevents byte-level interleaving such as
  // "VESC:STAT:...VESC:RX:..." which destroys both host parsers.
  if (!tx_message_active_) {
    tx_message_high_ = high_ready;
    tx_message_active_ = true;
  }
  bool use_high = tx_message_high_;
  if (use_high && !high_ready) {
    tx_message_active_ = false;
    return;
  }
  if (!use_high && !low_ready) {
    tx_message_active_ = false;
    return;
  }

  const uint8_t *ring = use_high ? tx_high_ : tx_;
  const uint16_t head = use_high ? tx_high_head_ : tx_head_;
  const uint16_t tail = use_high ? tx_high_tail_ : tx_tail_;
  static_assert(kHighTxSize == kTxSize,
                "USB CDC priority rings must share wrap size");
  const uint16_t size = kTxSize;
  const uint16_t used = RingUsed(head, tail, size);
  const uint16_t contiguous = head > tail ? static_cast<uint16_t>(head - tail)
                                          : static_cast<uint16_t>(size - tail);
  uint16_t count = std::min<uint16_t>(64U, std::min(used, contiguous));
  bool ends_message = false;
  for (uint16_t i = 0U; i < count; ++i) {
    if (ring[static_cast<uint16_t>((tail + i) % size)] == '\n') {
      count = static_cast<uint16_t>(i + 1U);
      ends_message = true;
      break;
    }
  }
  std::memcpy(tx_packet_, &ring[tail], count);
  if (USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_packet_, count) != USBD_OK)
    return;
  tx_pending_ = count;
  tx_active_high_ = use_high;
  tx_packet_ends_message_ = ends_message;
  tx_busy_ = true;
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) {
    tx_busy_ = false;
    tx_pending_ = 0U;
    tx_active_high_ = false;
    tx_packet_ends_message_ = false;
  }
}

void UsbCdcPort::flush(uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  while ((tx_head_ != tx_tail_ || tx_high_head_ != tx_high_tail_ || tx_busy_) &&
         static_cast<uint32_t>(HAL_GetTick() - start) < timeout_ms) {
    poll();
  }
}

void UsbCdcPort::onReceive(const uint8_t *data, uint32_t length) {
  if (data == nullptr)
    return;
  for (uint32_t i = 0; i < length; ++i) {
    const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxSize);
    if (next == rx_tail_) {
      ++rx_dropped_;
      break;
    }
    rx_[rx_head_] = data[i];
    rx_head_ = next;
  }
}

void UsbCdcPort::onTransmitComplete() {
  if (!tx_busy_)
    return;
  if (tx_active_high_) {
    tx_high_tail_ =
        static_cast<uint16_t>((tx_high_tail_ + tx_pending_) % kHighTxSize);
  } else {
    tx_tail_ = static_cast<uint16_t>((tx_tail_ + tx_pending_) % kTxSize);
  }
  const bool message_done = tx_packet_ends_message_;
  tx_pending_ = 0U;
  tx_busy_ = false;
  tx_active_high_ = false;
  tx_packet_ends_message_ = false;
  if (message_done)
    tx_message_active_ = false;
  // Start the next packet immediately from USB completion context so P1 motor
  // telemetry is not dependent on TFT/GNSS main-loop latency.
  poll();
}
