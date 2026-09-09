#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID 0x0483U
#define USBD_PID 0x5741U
#define USBD_LANGID_STRING 0x0409U
#define USBD_MANUFACTURER_STRING "STMicroelectronics"
#define USBD_PRODUCT_STRING "BLACKPILL_F411CE BOOT CDC"
#define USBD_CONFIGURATION_STRING "Boot CDC Config"
#define USBD_INTERFACE_STRING "Boot CDC Interface"

static uint8_t *DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *LangIDDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *ManufacturerDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *ProductDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *SerialDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *ConfigDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *InterfaceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static void IntToUnicode(uint32_t value, uint8_t *buffer, uint8_t length);
static void GetSerialNumber(void);

USBD_DescriptorsTypeDef USBD_Desc = {
  DeviceDescriptor, LangIDDescriptor, ManufacturerDescriptor, ProductDescriptor,
  SerialDescriptor, ConfigDescriptor, InterfaceDescriptor
};

__ALIGN_BEGIN static uint8_t DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12U, USB_DESC_TYPE_DEVICE, 0x00U, 0x02U,
  0x02U, 0x02U, 0x00U, USB_MAX_EP0_SIZE,
  LOBYTE(USBD_VID), HIBYTE(USBD_VID), LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00U, 0x02U, USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR, USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t SerialString[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL, USB_DESC_TYPE_STRING
};
__ALIGN_BEGIN static uint8_t StringDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length = sizeof(DeviceDesc); return DeviceDesc;
}
static uint8_t *LangIDDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length = sizeof(LangIDDesc); return LangIDDesc;
}
static uint8_t *ManufacturerDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, StringDesc, length); return StringDesc;
}
static uint8_t *ProductDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, StringDesc, length); return StringDesc;
}
static uint8_t *SerialDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; GetSerialNumber(); *length = USB_SIZ_STRING_SERIAL; return SerialString;
}
static uint8_t *ConfigDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING, StringDesc, length); return StringDesc;
}
static uint8_t *InterfaceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)USBD_INTERFACE_STRING, StringDesc, length); return StringDesc;
}

static void GetSerialNumber(void) {
  uint32_t serial0 = *(uint32_t *)DEVICE_ID1;
  const uint32_t serial1 = *(uint32_t *)DEVICE_ID2;
  const uint32_t serial2 = *(uint32_t *)DEVICE_ID3;
  serial0 += serial2;
  if (serial0 != 0U) {
    IntToUnicode(serial0, &SerialString[2], 8U);
    IntToUnicode(serial1, &SerialString[18], 4U);
  }
}
static void IntToUnicode(uint32_t value, uint8_t *buffer, uint8_t length) {
  for (uint8_t i = 0U; i < length; ++i) {
    const uint8_t nibble = (uint8_t)(value >> 28U);
    buffer[2U * i] = (uint8_t)(nibble < 10U ? (uint32_t)(nibble + (uint32_t)'0') : (uint32_t)(nibble - 10U + (uint32_t)'A'));
    buffer[2U * i + 1U] = 0U;
    value <<= 4U;
  }
}
