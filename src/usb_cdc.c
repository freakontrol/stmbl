#include "usbd_cdc_if.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usb_dcd_int.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define RX_QUEUE_SIZE 512

struct ringbuf usb_rx_buf = {.buf = (char[RX_QUEUE_SIZE]){0}, .bufsize = RX_QUEUE_SIZE};


static USB_OTG_CORE_HANDLE USB_OTG_dev;

static uint16_t VCP_Init(void) {
  return USBD_OK;
}

static uint16_t VCP_DeInit(void) {
  return USBD_OK;
}

static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t *Buf, uint32_t Len) {
  return USBD_OK;
}

// this function is not called
static uint16_t VCP_DataTx(void) {
  return USBD_OK;
}

static uint16_t VCP_DataRx(uint8_t *buf, uint32_t len) {
  rb_write(&usb_rx_buf, buf, len);
  return USBD_OK;
}

const CDC_IF_Prop_TypeDef VCP_fops = {
    .pIf_Init   = VCP_Init,
    .pIf_DeInit = VCP_DeInit,
    .pIf_Ctrl   = VCP_Ctrl,
    .pIf_DataTx = VCP_DataTx,
    .pIf_DataRx = VCP_DataRx};

void usb_init(void) {
  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
}

void OTG_FS_IRQHandler(void) {
  USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev) {
  // Enable peripheral clocks
  //
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

  // enable I/O compensation cell to reduce the I/O noise on power supply
  SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;

  // Configure DM and DP Pins
  //
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);

  GPIO_Init(GPIOA, &(GPIO_InitTypeDef){.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12, .GPIO_Speed = GPIO_Speed_50MHz, .GPIO_Mode = GPIO_Mode_AF, .GPIO_OType = GPIO_OType_PP, .GPIO_PuPd = GPIO_PuPd_NOPULL});
}

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev) {
  NVIC_Init(&(NVIC_InitTypeDef){
      .NVIC_IRQChannel                   = OTG_FS_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 15,
      .NVIC_IRQChannelSubPriority        = 0,
      .NVIC_IRQChannelCmd                = ENABLE});
}


void USB_OTG_BSP_uDelay(const uint32_t usec) {
  uint32_t count       = 0;
  const uint32_t utime = (120 * usec / 7);
  do {
    if(++count > utime) {
      return;
    }
  } while(1);
}


void USB_OTG_BSP_mDelay(const uint32_t msec) {
  USB_OTG_BSP_uDelay(msec * 1000);
}

// USB_Usr

enum {
  USB_CDC_DETACHED,
  USB_CDC_CONNECTED
} E_USB_STAT;

uint8_t usb_cdc_status = USB_CDC_DETACHED;

void USBD_USR_Init(void) {
  usb_cdc_status = USB_CDC_DETACHED;
}

void USBD_USR_DeviceReset(uint8_t speed) {
  usb_cdc_status = USB_CDC_DETACHED;
}

void USBD_USR_DeviceConfigured(void) {
  usb_cdc_status = USB_CDC_CONNECTED;
}

void USBD_USR_DeviceConnected(void) {
  usb_cdc_status = USB_CDC_DETACHED;
}

void USBD_USR_DeviceDisconnected(void) {
  usb_cdc_status = USB_CDC_DETACHED;
}

void USBD_USR_DeviceSuspended(void) {
  usb_cdc_status = USB_CDC_DETACHED;
}

void USBD_USR_DeviceResumed(void) {
  usb_cdc_status = USB_CDC_DETACHED;
}

USBD_Usr_cb_TypeDef USR_cb =
    {
        USBD_USR_Init,
        USBD_USR_DeviceReset,
        USBD_USR_DeviceConfigured,
        USBD_USR_DeviceSuspended,
        USBD_USR_DeviceResumed,
        USBD_USR_DeviceConnected,
        USBD_USR_DeviceDisconnected,
};

uint8_t USB_CDC_is_connected(void) {
  return usb_cdc_status;
}

//TODO: implement new term API
void cdc_init(void) {}

extern uint32_t APP_Rx_ptr_out;

// Free bytes in the IN ring. APP_Rx_ptr_out belongs to the USB interrupt
// (Handle_USBAsynchXfer), which leaves it at APP_RX_DATA_SIZE after draining
// to the end and only folds it back to 0 on its next pass, so that value means
// 0 here. One byte stays unused: in == out has to mean empty.
static uint32_t cdc_tx_room(void) {
  uint32_t out = *(volatile uint32_t *)&APP_Rx_ptr_out;
  if(out >= APP_RX_DATA_SIZE) {
    out = 0;
  }
  uint32_t used = (APP_Rx_ptr_in + APP_RX_DATA_SIZE - out) % APP_RX_DATA_SIZE;
  return APP_RX_DATA_SIZE - 1 - used;
}

// Copy len bytes into the IN ring and publish them with a single store of
// APP_Rx_ptr_in, so the interrupt never sees a half folded index. Callers
// check the room first: lapping unsent bytes corrupts the host's demux.
static void cdc_tx_put(const uint8_t *data, uint32_t len) {
  uint32_t in = APP_Rx_ptr_in;
  while(len--) {
    APP_Rx_Buffer[in] = *data++;
    in                = in + 1 >= APP_RX_DATA_SIZE ? 0 : in + 1;
  }
  __asm__ volatile("" ::: "memory");  // the bytes land before the index moves
  *(volatile uint32_t *)&APP_Rx_ptr_in = in;
}

// Scope packets go whole or not at all: a partial packet desyncs the host, a
// missing one is just a gap.
int cdc_tx(void *data, uint32_t len) {
  if(!cdc_is_connected() || cdc_tx_room() < len) {
    return 0;
  }
  cdc_tx_put((const uint8_t *)data, len);
  return len;
}

// Text waits a little for the interrupt to drain, then drops what does not
// fit rather than overwriting what has not been sent.
int cdc_tx_text(const char *data, int len) {
  int sent = 0;
  for(uint32_t spin = 0; len > 0 && spin < 200000; spin++) {
    uint32_t room = cdc_tx_room();
    if(room == 0) {
      continue;
    }
    uint32_t n = (uint32_t)len < room ? (uint32_t)len : room;
    cdc_tx_put((const uint8_t *)data + sent, n);
    sent += n;
    len -= n;
  }
  return sent;
}

// usb_rx_buf is filled from the USB interrupt and ringbuf.c has no locking,
// which merged and lost commands: hold the interrupt off while reading
int cdc_getline(char *ptr, int len) {
  NVIC_DisableIRQ(OTG_FS_IRQn);
  int ret = rb_getline(&usb_rx_buf, ptr, len);
  NVIC_EnableIRQ(OTG_FS_IRQn);
  return ret;
}

int cdc_is_connected() {
  return USB_CDC_is_connected();
}

void cdc_poll() {}
