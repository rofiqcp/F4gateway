#include "boot_usb.h"
#if defined(BOARD_F103_BOOT)
#include "stm32f1xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

#define BOOT_RX_SIZE 2048U

#if defined(BOARD_F103_BOOT)
static void reset_usb_peripheral_while_detached(void) {
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
  __HAL_RCC_USB_FORCE_RESET();
  __DSB();
  for (volatile uint32_t i = 0U; i < 64U; ++i) __NOP();
  __HAL_RCC_USB_RELEASE_RESET();
  __HAL_RCC_USB_CLK_DISABLE();
  __DSB();
}

static void force_usb_disconnect_pulse(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  reset_usb_peripheral_while_detached();
  HAL_Delay(500U);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
}
#endif

static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;
static volatile uint32_t rx_dropped = 0U;
static uint8_t rx_buffer[BOOT_RX_SIZE];
static uint8_t tx_packet[64];

bool boot_usb_begin(void) {
#if defined(BOARD_F103_BOOT)
  force_usb_disconnect_pulse();
#endif
  rx_head = rx_tail = 0U;
  rx_dropped = 0U;

  if (USBD_Init(&hUsbDeviceFS, &USBD_Desc, 0U) != USBD_OK)
    return false;
  if (USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS) != USBD_OK) {
    (void)USBD_DeInit(&hUsbDeviceFS);
    return false;
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
    (void)USBD_DeInit(&hUsbDeviceFS);
    return false;
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    (void)USBD_Stop(&hUsbDeviceFS);
    (void)USBD_DeInit(&hUsbDeviceFS);
    return false;
  }
  return true;
}

void boot_usb_end(void) {
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
}

void boot_usb_disconnect_hold(void) {
  boot_usb_end();
#if defined(BOARD_F103_BOOT)
  /*
   * Bluepill-class boards use an external D+ pull-up. Merely stopping the USB
   * peripheral does not guarantee a physical detach, so hold PA12 low across
   * the bootloader -> application handoff. The application startup pulse will
   * take ownership of PA12 and release it only after its own detach interval.
   */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  reset_usb_peripheral_while_detached();
#endif
}

uint32_t boot_usb_rx_dropped(void) { return rx_dropped; }

bool boot_usb_connected(void) {
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

int boot_usb_available(void) {
  const uint16_t head = rx_head;
  const uint16_t tail = rx_tail;
  return head >= tail ? (int)(head - tail) : (int)(BOOT_RX_SIZE - tail + head);
}

int boot_usb_read(void) {
  if (rx_head == rx_tail) return -1;
  const uint8_t value = rx_buffer[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % BOOT_RX_SIZE);
  return (int)value;
}

void boot_usb_on_receive(const uint8_t *data, uint32_t length) {
  if (data == 0) return;
  for (uint32_t i = 0U; i < length; ++i) {
    const uint16_t next = (uint16_t)((rx_head + 1U) % BOOT_RX_SIZE);
    if (next == rx_tail) { ++rx_dropped; break; }
    rx_buffer[rx_head] = data[i];
    rx_head = next;
  }
}

static bool wait_tx_idle(uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  while (1) {
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (cdc != 0 && cdc->TxState == 0U) return true;
    if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) return false;
    HAL_Delay(1U);
  }
}

static bool write_bytes(const uint8_t *data, uint32_t length, uint32_t timeout_ms) {
  uint32_t offset = 0U;
  while (offset < length) {
    if (!boot_usb_connected() || !wait_tx_idle(timeout_ms)) return false;
    const uint32_t chunk = (length - offset) > sizeof(tx_packet) ? sizeof(tx_packet) : (length - offset);
    memcpy(tx_packet, data + offset, chunk);
    if (USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_packet, chunk) != USBD_OK) return false;
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) return false;
    if (!wait_tx_idle(timeout_ms)) return false;
    offset += chunk;
  }
  return true;
}

bool boot_usb_write_line(const char *line, uint32_t timeout_ms) {
  if (line == 0) return false;
  const uint32_t len = (uint32_t)strlen(line);
  if (!write_bytes((const uint8_t *)line, len, timeout_ms)) return false;
  static const uint8_t eol[2] = {'\r', '\n'};
  return write_bytes(eol, 2U, timeout_ms);
}

void boot_usb_flush(uint32_t timeout_ms) {
  (void)wait_tx_idle(timeout_ms);
}
