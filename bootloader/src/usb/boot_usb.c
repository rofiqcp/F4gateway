#include "boot_usb.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

#define BOOT_RX_SIZE 2048U
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;
static volatile uint32_t rx_dropped = 0U;
static uint8_t rx_buffer[BOOT_RX_SIZE];
static uint8_t tx_packet[64];

bool boot_usb_begin(void) {
  rx_head = rx_tail = 0U;
  rx_dropped = 0U;
  if (USBD_Init(&hUsbDeviceFS, &USBD_Desc, 0U) != USBD_OK) return false;
  if (USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS) != USBD_OK) return false;
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) return false;
  return USBD_Start(&hUsbDeviceFS) == USBD_OK;
}

void boot_usb_end(void) {
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
}

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
