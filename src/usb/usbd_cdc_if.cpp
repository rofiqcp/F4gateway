#include "usbd_cdc.h"
#include "UsbCdcPort.h"

static uint8_t UserRxBufferFS[CDC_DATA_FS_OUT_PACKET_SIZE];
static uint8_t UserTxBufferFS[CDC_DATA_FS_IN_PACKET_SIZE];

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
#if !defined(BOARD_F103C8)
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
#endif

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
#if !defined(BOARD_F103C8)
  , CDC_TransmitCplt_FS
#endif
};

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void) {
  gUsb.onUsbClassInit();
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void) {
  gUsb.onUsbClassDeInit();
  return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  // CDC ACM line coding is informational for USB CDC (the transport itself is
  // USB, not a hardware UART). Still implement the mandatory requests so host
  // drivers see a standards-compliant ACM endpoint. Default: 1,000,000 8N1.
  static uint8_t line_coding[7] = {0x40U, 0x42U, 0x0FU, 0x00U, 0x00U, 0x00U, 0x08U};
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      if (pbuf != nullptr && length >= sizeof(line_coding)) {
        for (uint8_t i = 0U; i < sizeof(line_coding); ++i) line_coding[i] = pbuf[i];
      }
      break;
    case CDC_GET_LINE_CODING:
      if (pbuf != nullptr && length >= sizeof(line_coding)) {
        for (uint8_t i = 0U; i < sizeof(line_coding); ++i) pbuf[i] = line_coding[i];
      }
      break;
    case CDC_SET_CONTROL_LINE_STATE:
    default:
      break;
  }
  return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len) {
  gUsb.onReceive(pbuf, *Len);
  const bool armed =
      USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS) == USBD_OK &&
      USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK;
  if (!armed)
    gUsb.onRxRearmFailure();
  return (int8_t)USBD_OK;
}

// Retry a failed OUT endpoint arm from the cooperative main-loop service.
// Never call physical USB detach here: endpoint repair must keep ttyACM alive.
extern "C" bool F4Gateway_CdcTryRearmRxFromMain(void) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
      hUsbDeviceFS.pClassData == nullptr)
    return false;
  if (USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS) != USBD_OK)
    return false;
  return USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK;
}

#if !defined(BOARD_F103C8)
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum) {
  (void)pbuf;
  (void)Len;
  (void)epnum;
  gUsb.onTransmitComplete();
  return (int8_t)USBD_OK;
}
#else
// Older STM32CubeF1 CDC middleware has no TransmitCplt member in the class
// interface. usbd_conf.c calls this bridge after EP1 IN completion.
extern "C" void F4Gateway_CdcTxCompleteFromIsr(void) {
  gUsb.onTransmitComplete();
}
#endif
