#if defined(BOARD_F103_BOOT)
#include "stm32f1xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "usbd_core.h"
#include "usbd_cdc.h"

#if defined(BOARD_F103_BOOT)
PCD_HandleTypeDef hpcd_USB_FS;
#define HUSB_PCD hpcd_USB_FS
#else
PCD_HandleTypeDef hpcd_USB_OTG_FS;
#define HUSB_PCD hpcd_USB_OTG_FS
#endif
extern USBD_HandleTypeDef hUsbDeviceFS;

void *USBD_static_malloc(uint32_t size) {
  static uint32_t cdc_mem[(sizeof(USBD_CDC_HandleTypeDef) + 3U) / 4U];
  return size <= sizeof(cdc_mem) ? cdc_mem : NULL;
}
void USBD_static_free(void *p) { (void)p; }

#if defined(BOARD_F103_BOOT)
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state) {
  if (hpcd == NULL || hpcd->Instance != USB) return;
  hpcd->Instance->DADDR = state != 0U ? USB_DADDR_EF : 0U;
}
#endif

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
#if defined(BOARD_F103_BOOT)
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#else
  if (hpcd->Instance != USB_OTG_FS) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_SetPriority(OTG_FS_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
#endif
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
#if defined(BOARD_F103_BOOT)
  if (hpcd->Instance != USB) return;
  __HAL_RCC_USB_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
#else
  if (hpcd->Instance != USB_OTG_FS) return;
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
#endif
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
  USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData); }

#if defined(BOARD_F103_BOOT)
void USB_LP_CAN1_RX0_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }
#else
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
#endif

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
#if defined(BOARD_F103_BOOT)
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8U;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = 64U;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) return USBD_FAIL;
  // PMA layout: EP0 OUT/IN, CDC data IN/OUT, CDC command IN.
  if (HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00U, PCD_SNG_BUF, 0x18U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80U, PCD_SNG_BUF, 0x58U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81U, PCD_SNG_BUF, 0xC0U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x01U, PCD_SNG_BUF, 0x110U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82U, PCD_SNG_BUF, 0x100U) != HAL_OK) return USBD_FAIL;
#else
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4U;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_FS;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 128U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0U, 64U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1U, 96U) != HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2U, 32U) != HAL_OK) return USBD_FAIL;
#endif
  return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
  return HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
  return HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
  return HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
  return HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr) == HAL_OK ? USBD_OK : USBD_FAIL;
}
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  return (ep_addr & 0x80U) != 0U ? hpcd->IN_ep[ep_addr & 0x7FU].is_stall
                                 : hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
  return HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr) == HAL_OK ? USBD_OK : USBD_FAIL;
}
#if defined(BOARD_F103_BOOT)
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  return HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  return HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK ? USBD_OK : USBD_FAIL;
}
#else
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
  return HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
  return HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size) == HAL_OK ? USBD_OK : USBD_FAIL;
}
#endif
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}
void USBD_LL_Delay(uint32_t Delay) { HAL_Delay(Delay); }
