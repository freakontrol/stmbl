// Glue between the STM32 USB Device Library and the HAL PCD driver (OTG FS)

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

PCD_HandleTypeDef hpcd_fs;

void cdc_sof(void);

// Also entered by cdc_tx*() pending the IRQ, so new data starts sending right
// away instead of on the next SOF, always from this one interrupt context.
void OTG_FS_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_fs);
  cdc_sof();
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
  // Enable peripheral clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // needed for SYSCFG->CMPCR below
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

  // enable I/O compensation cell to reduce the I/O noise on power supply
  SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;

  // DM and DP, no VBUS sensing, no ID pin
  LL_GPIO_Init(GPIOA, &(LL_GPIO_InitTypeDef){.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12, .Speed = LL_GPIO_SPEED_FREQ_HIGH, .Mode = LL_GPIO_MODE_ALTERNATE, .OutputType = LL_GPIO_OUTPUT_PUSHPULL, .Pull = LL_GPIO_PULL_NO, .Alternate = LL_GPIO_AF_10});

  NVIC_SetPriority(OTG_FS_IRQn, 15);
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
  NVIC_DisableIRQ(OTG_FS_IRQn);
  RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
}

// PCD -> USB Device Library

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SOF(hpcd->pData);
  cdc_sof();
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
  USBD_LL_Reset(hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Suspend(hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Resume(hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_DevConnected(hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_DevDisconnected(hpcd->pData);
}

// USB Device Library -> PCD

static USBD_StatusTypeDef status(HAL_StatusTypeDef s) {
  switch(s) {
    case HAL_OK:
      return USBD_OK;
    case HAL_BUSY:
      return USBD_BUSY;
    default:
      return USBD_FAIL;
  }
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
  hpcd_fs.Instance                 = USB_OTG_FS;
  hpcd_fs.Init.dev_endpoints       = 4;
  hpcd_fs.Init.dma_enable          = 0;
  hpcd_fs.Init.low_power_enable    = 0;
  hpcd_fs.Init.lpm_enable          = 0;
  hpcd_fs.Init.phy_itface          = PCD_PHY_EMBEDDED;
  hpcd_fs.Init.Sof_enable          = 1;  // cdc_sof() starts IN transfers every frame
  hpcd_fs.Init.speed               = PCD_SPEED_FULL;
  hpcd_fs.Init.vbus_sensing_enable = 0;
  hpcd_fs.Init.use_dedicated_ep1   = 0;
  hpcd_fs.pData                    = pdev;
  pdev->pData                      = &hpcd_fs;
  if(HAL_PCD_Init(&hpcd_fs) != HAL_OK) {
    return USBD_FAIL;
  }
  // 320 words of FIFO RAM on OTG FS
  HAL_PCDEx_SetRxFiFo(&hpcd_fs, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_fs, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_fs, 1, 0x80);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
  return status(HAL_PCD_DeInit(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
  return status(HAL_PCD_Start(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
  return status(HAL_PCD_Stop(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
  return status(HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return status(HAL_PCD_EP_Close(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return status(HAL_PCD_EP_Flush(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return status(HAL_PCD_EP_SetStall(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return status(HAL_PCD_EP_ClrStall(pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  PCD_HandleTypeDef *hpcd = pdev->pData;
  if((ep_addr & 0x80U) == 0x80U) {
    return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
  }
  return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
  return status(HAL_PCD_SetAddress(pdev->pData, dev_addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
  return status(HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
  return status(HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay) {
  HAL_Delay(Delay);
}

// the CDC class allocates its handle once, at SetConfig
void *USBD_static_malloc(uint32_t size) {
  static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4) + 1];
  return size <= sizeof(mem) ? mem : NULL;
}

void USBD_static_free(void *p) {
}
