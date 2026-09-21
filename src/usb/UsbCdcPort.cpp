#include "UsbCdcPort.h"
#include "BoardSupport.h"

#include "McuHal.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"

#include <algorithm>
#include <cstring>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

UsbCdcPort gUsb;

namespace {
#if defined(BOARD_F103C8)
void ForceUsbDisconnectPulse() {
  // Blue Pill boards normally expose USB D+ through an external pull-up.
  // Driving PA12 low briefly guarantees the host observes a real disconnect
  // after MCU reset or a software USB recovery before the USB peripheral owns
  // D+ again.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}
#endif

uint16_t RingUsed(uint16_t head, uint16_t tail, uint16_t size) {
  return head >= tail ? static_cast<uint16_t>(head - tail)
                      : static_cast<uint16_t>(size - tail + head);
}
uint16_t RingFree(uint16_t head, uint16_t tail, uint16_t size) {
  return static_cast<uint16_t>(size - RingUsed(head, tail, size) - 1U);
}
} // namespace

void UsbCdcPort::resetSessionState(bool drop_queues, bool count_abort) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (count_abort && tx_busy_)
    ++tx_abort_on_session_reset_;
  tx_busy_ = false;
  tx_active_high_ = false;
  tx_message_active_ = false;
  tx_message_high_ = false;
  tx_packet_ends_message_ = false;
  tx_pending_ = 0U;
  tx_started_ms_ = 0U;
  tx_stall_reported_ = false;
  tx_service_pending_ = false;
#ifdef HMI_TEST_HOOKS
  test_suppress_tx_until_ms_ = 0U;
#endif
  if (drop_queues) {
    rx_head_ = rx_tail_ = 0U;
    rx_discard_until_newline_ = false;
    tx_head_ = tx_tail_ = 0U;
    tx_high_head_ = tx_high_tail_ = 0U;
    // Every destructive queue reset advances the safety transport epoch. This
    // includes endpoint split-brain repair, not only USB class re-enumeration.
    ++usb_session_generation_;
  }
  if (primask == 0U)
    __enable_irq();
}

bool UsbCdcPort::startUsbStack() {
  if (USBD_Init(&hUsbDeviceFS, &USBD_Desc, 0U) != USBD_OK)
    return false;
  if (USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS) != USBD_OK)
    return false;
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
    return false;
  return USBD_Start(&hUsbDeviceFS) == USBD_OK;
}

bool UsbCdcPort::begin() {
#if defined(BOARD_F103C8)
  ForceUsbDisconnectPulse();
#endif
  resetSessionState(true, false);
  rx_dropped_ = tx_dropped_ = 0U;
  tx_low_dropped_ = tx_high_dropped_ = 0U;
  rx_high_water_ = tx_low_high_water_ = tx_high_high_water_ = 0U;
  tx_complete_ms_ = HAL_GetTick();
  usb_session_generation_ = 0U;
  usb_class_init_count_ = 0U;
  usb_class_deinit_count_ = 0U;
  tx_abort_on_session_reset_ = 0U;
  tx_complete_count_ = 0U;
  tx_stall_recovery_count_ = 0U;
  usb_soft_restart_count_ = 0U;
  last_rx_ms_ = 0U;
  rx_packet_count_ = 0U;
  rx_resync_count_ = 0U;
  rx_resync_complete_count_ = 0U;
  rx_discard_until_newline_ = false;
  tx_progress_stall_count_ = 0U;
  last_recovery_reason_ = 0U;
  last_repair_age_ms_ = 0U;
  last_repair_pending_ = 0U;
  last_repair_ep_length_ = 0U;
  last_repair_flags_ = 0U;
  tx_stall_reported_ = false;
  recovery_pending_ = false;
  host_session_token_ = 0U;
  host_session_count_ = 0U;
  host_session_established_ = false;
  low_session_purge_count_ = 0U;
  high_session_purge_count_ = 0U;
  purge_low_after_message_ = false;
  return startUsbStack();
}

void UsbCdcPort::end() {
  invalidateHostSession();
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
  resetSessionState(true, true);
}

void UsbCdcPort::onUsbClassInit() {
  ++usb_class_init_count_;
  invalidateHostSession();
  resetSessionState(true, true);
  tx_complete_ms_ = HAL_GetTick();
}

void UsbCdcPort::onUsbClassDeInit() {
  ++usb_class_deinit_count_;
  invalidateHostSession();
  resetSessionState(true, true);
}

void UsbCdcPort::requestRecovery() {
  last_recovery_reason_ = 1U;
  recovery_pending_ = true;
}

void UsbCdcPort::beginHostSession(uint32_t token) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  host_session_established_ = false;
  host_session_token_ = token;
  ++host_session_count_;

  // A new host epoch must not inherit queued records from the previous host.
  // If a message is already physically in flight, preserve only that ONE complete
  // newline-delimited message, then append the new session ACK behind it. The new
  // host intentionally ignores pre-ACK bytes, so this preserves line framing
  // without allowing stale high-priority control backlog to cross epochs.
  const auto trim_after_active_message = [](const uint8_t *ring, volatile uint16_t &head,
                                            volatile uint16_t &tail, uint16_t size,
                                            bool active) -> bool {
    if (!active) {
      const bool had_data = head != tail;
      head = tail;
      return had_data;
    }
    uint16_t cursor = tail;
    while (cursor != head) {
      const uint8_t value = ring[cursor];
      cursor = static_cast<uint16_t>((cursor + 1U) % size);
      if (value == '\n') {
        const bool purged = cursor != head;
        head = cursor;
        return purged;
      }
    }
    // Raw writers should still be newline-framed. If not, leave the active data
    // intact rather than truncating a USB packet into an unrecoverable fragment.
    return false;
  };

  if (trim_after_active_message(tx_, tx_head_, tx_tail_, kTxSize,
                                tx_message_active_ && !tx_message_high_))
    ++low_session_purge_count_;
  if (trim_after_active_message(tx_high_, tx_high_head_, tx_high_tail_, kHighTxSize,
                                tx_message_active_ && tx_message_high_))
    ++high_session_purge_count_;
  purge_low_after_message_ = false;

  if (primask == 0U) __enable_irq();
}

void UsbCdcPort::confirmHostSession() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (host_session_token_ != 0U && connected()) host_session_established_ = true;
  if (primask == 0U) __enable_irq();
}

void UsbCdcPort::invalidateHostSession() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  host_session_established_ = false;
  host_session_token_ = 0U;
  if (primask == 0U) __enable_irq();
}

uint32_t UsbCdcPort::txBusyAgeMs() const {
  return tx_busy_ && tx_started_ms_ != 0U
             ? static_cast<uint32_t>(HAL_GetTick() - tx_started_ms_)
             : 0U;
}

uint32_t UsbCdcPort::lastTxCompleteAgeMs() const {
  return tx_complete_ms_ != 0U
             ? static_cast<uint32_t>(HAL_GetTick() - tx_complete_ms_)
             : 0xFFFFFFFFUL;
}

uint32_t UsbCdcPort::lastRxAgeMs() const {
  return last_rx_ms_ != 0U ? static_cast<uint32_t>(HAL_GetTick() - last_rx_ms_)
                           : 0xFFFFFFFFUL;
}

uint16_t UsbCdcPort::rxQueueDepth() const {
  return RingUsed(rx_head_, rx_tail_, kRxSize);
}
uint16_t UsbCdcPort::txLowQueueDepth() const {
  return RingUsed(tx_head_, tx_tail_, kTxSize);
}
uint16_t UsbCdcPort::txHighQueueDepth() const {
  return RingUsed(tx_high_head_, tx_high_tail_, kHighTxSize);
}

uint32_t UsbCdcPort::cdcTxState() const {
  auto *hcdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
  return hcdc != nullptr ? hcdc->TxState : 0xFFFFFFFFUL;
}

bool UsbCdcPort::softRestartUsb() {
  ++usb_soft_restart_count_;
  invalidateHostSession();
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
  resetSessionState(true, true);
  HAL_Delay(10U);
  return startUsbStack();
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
    ++tx_low_dropped_;
    if (primask == 0U)
      __enable_irq();
    return 0U;
  }
  for (std::size_t i = 0; i < length; ++i) {
    tx_[tx_head_] = data[i];
    tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  }
  { const uint16_t used = RingUsed(tx_head_, tx_tail_, kTxSize); if (used > tx_low_high_water_) tx_low_high_water_ = used; }
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
    ++tx_high_dropped_;
    if (primask == 0U)
      __enable_irq();
    return 0U;
  }
  for (std::size_t i = 0; i < length; ++i) {
    tx_high_[tx_high_head_] = data[i];
    tx_high_head_ = static_cast<uint16_t>((tx_high_head_ + 1U) % kHighTxSize);
  }
  { const uint16_t used = RingUsed(tx_high_head_, tx_high_tail_, kHighTxSize); if (used > tx_high_high_water_) tx_high_high_water_ = used; }
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
    ++tx_low_dropped_;
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
  { const uint16_t used = RingUsed(tx_head_, tx_tail_, kTxSize); if (used > tx_low_high_water_) tx_low_high_water_ = used; }
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
    ++tx_high_dropped_;
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
  { const uint16_t used = RingUsed(tx_high_head_, tx_high_tail_, kHighTxSize); if (used > tx_high_high_water_) tx_high_high_water_ = used; }
  if (primask == 0U)
    __enable_irq();
  poll();
  return true;
}

bool UsbCdcPort::writeLineCritical(const char *line, uint32_t timeout_ms) {
  (void)timeout_ms;
  if (line == nullptr || !connected())
    return false;
  // Critical means high-priority, not permission to block the F411 main context.
  // Give the endpoint state machine one cooperative service opportunity and then
  // fail closed. Safety STOP messages have their own persistent retry latch in
  // main.cpp; maintenance commands can simply be retried by the operator/host.
  if (writeLineHighPriority(line))
    return true;
  service();
  return writeLineHighPriority(line);
}

#ifdef HMI_TEST_HOOKS
void UsbCdcPort::testSuppressTx(uint32_t duration_ms) {
  test_suppress_tx_until_ms_ = HAL_GetTick() + duration_ms;
}
#endif

void UsbCdcPort::service() {
  const bool explicit_recovery = recovery_pending_;
  recovery_pending_ = false;

  // TX-complete runs in USB IRQ context. Revalidate wrapper + ST class state
  // atomically; otherwise the ISR can complete between the first tx_busy read
  // and the TxState check, producing a false split-brain recovery.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (connected() && tx_busy_ && tx_started_ms_ != 0U) {
    const uint32_t age = static_cast<uint32_t>(HAL_GetTick() - tx_started_ms_);
    if (age >= kTxStallRepairMs) {
      if (!tx_stall_reported_) {
        tx_stall_reported_ = true;
        ++tx_progress_stall_count_;
      }
      auto *hcdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
      // Only repair an impossible split-brain state: wrapper busy while the ST
      // CDC class is already idle. TxState==1 is legitimate host backpressure.
      if (hcdc != nullptr && hcdc->TxState == 0U && tx_busy_) {
        ++tx_stall_recovery_count_;
        last_recovery_reason_ = 2U;
        last_repair_age_ms_ = age;
        last_repair_pending_ = tx_pending_;
        last_repair_ep_length_ = hUsbDeviceFS.ep_in[CDC_IN_EP & 0x0FU].total_length;
        last_repair_flags_ = static_cast<uint8_t>((tx_active_high_ ? 1U : 0U) |
                                                  (tx_packet_ends_message_ ? 2U : 0U));
        resetSessionState(true, true);
      }
    }
  }
  if (primask == 0U)
    __enable_irq();

  if (explicit_recovery)
    (void)softRestartUsb();
  // TX completion IRQ only updates ring state and raises this event. All calls
  // into the ST USB transmit stack occur here in thread/main-loop context.
  tx_service_pending_ = false;
  poll();
}

void UsbCdcPort::poll() {
#ifdef HMI_TEST_HOOKS
  if (test_suppress_tx_until_ms_ != 0U) {
    if (static_cast<int32_t>(test_suppress_tx_until_ms_ - HAL_GetTick()) > 0)
      return;
    test_suppress_tx_until_ms_ = 0U;
  }
#endif
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
  tx_started_ms_ = HAL_GetTick();
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) {
    tx_busy_ = false;
    tx_pending_ = 0U;
    tx_started_ms_ = 0U;
    tx_active_high_ = false;
    tx_packet_ends_message_ = false;
  }
}

void UsbCdcPort::flush(uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  while ((tx_head_ != tx_tail_ || tx_high_head_ != tx_high_tail_ || tx_busy_) &&
         static_cast<uint32_t>(HAL_GetTick() - start) < timeout_ms) {
    // A flush is only a bounded drain request, never permission to starve the
    // realtime loop. Progress CDC TX in thread context, then yield through the
    // board callback so STOP retries, HMI lease and safety I/O keep receiving
    // service even during the final DFU acknowledgement drain.
    service();
    Board_RealtimeService();
    if (tx_head_ != tx_tail_ || tx_high_head_ != tx_high_tail_ || tx_busy_)
      __WFI();
  }
}

void UsbCdcPort::onReceive(const uint8_t *data, uint32_t length) {
  if (data == nullptr)
    return;
  last_rx_ms_ = HAL_GetTick();
  ++rx_packet_count_;
  for (uint32_t i = 0; i < length; ++i) {
    const uint8_t value = data[i];
    if (rx_discard_until_newline_) {
      ++rx_dropped_;
      if (value == '\n') {
        rx_discard_until_newline_ = false;
        ++rx_resync_complete_count_;
      }
      continue;
    }
    const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxSize);
    if (next == rx_tail_) {
      // A ring overflow destroys line framing. Fail closed: purge every buffered
      // RX byte and discard input until the next newline before accepting a new
      // command. main.cpp observes rxResyncCount() and clears its partial parser
      // buffer as well, so bytes from opposite sides of the overflow can never
      // be concatenated into an accidental command.
      rx_tail_ = rx_head_;
      rx_discard_until_newline_ = true;
      ++rx_resync_count_;
      ++rx_dropped_;
      if (value == '\n') {
        rx_discard_until_newline_ = false;
        ++rx_resync_complete_count_;
      }
      continue;
    }
    rx_[rx_head_] = value;
    rx_head_ = next;
  }
  { const uint16_t used = RingUsed(rx_head_, rx_tail_, kRxSize); if (used > rx_high_water_) rx_high_water_ = used; }
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
  const bool completed_high = tx_active_high_;
  tx_pending_ = 0U;
  tx_busy_ = false;
  tx_started_ms_ = 0U;
  tx_stall_reported_ = false;
  tx_complete_ms_ = HAL_GetTick();
  ++tx_complete_count_;
  tx_active_high_ = false;
  tx_packet_ends_message_ = false;
  if (message_done) {
    tx_message_active_ = false;
    if (!completed_high && purge_low_after_message_) {
      tx_tail_ = tx_head_;
      purge_low_after_message_ = false;
      ++low_session_purge_count_;
    }
  }
  // Never call USBD_CDC_TransmitPacket() recursively from the USB IRQ. Mark a
  // service event; main-loop gUsb.service() starts the next packet.
  tx_service_pending_ = true;
}
