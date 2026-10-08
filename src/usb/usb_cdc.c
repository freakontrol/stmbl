// CDC virtual com port on the STM32 USB Device Library (Cube, HAL PCD)

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "stm32f4xx.h"

#define RX_QUEUE_SIZE 512

struct ringbuf usb_rx_buf = {.buf = (char[RX_QUEUE_SIZE]){0}, .bufsize = RX_QUEUE_SIZE};

static USBD_HandleTypeDef usb_dev;

// OUT endpoint packet buffer, handed back to the class after each packet
static uint8_t rx_packet[CDC_DATA_FS_OUT_PACKET_SIZE];

// IN ring, written by cdc_tx*() and sent from the USB interrupt. tx_out is
// only moved by the interrupt, once the transfer from it has completed.
static uint8_t tx_buf[APP_RX_DATA_SIZE];
static volatile uint32_t tx_in;
static volatile uint32_t tx_out;
static uint32_t tx_busy_len;  // bytes of the running IN transfer, 0 if none

static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};  // 115200 8N1

// Start the next IN transfer: the contiguous part of the ring from tx_out.
// Runs in the USB interrupt only (SOF and transfer complete).
static void tx_kick(void) {
  if(tx_busy_len || usb_dev.dev_state != USBD_STATE_CONFIGURED) {
    return;
  }
  uint32_t in  = tx_in;
  uint32_t out = tx_out;
  if(in == out) {
    return;
  }
  uint32_t len = (in > out ? in : APP_RX_DATA_SIZE) - out;
  USBD_CDC_SetTxBuffer(&usb_dev, &tx_buf[out], len);
  if(USBD_CDC_TransmitPacket(&usb_dev) == USBD_OK) {
    tx_busy_len = len;
  }
}

// SOF, and after every USB interrupt (see OTG_FS_IRQHandler)
void cdc_sof(void) {
  tx_kick();
}

static int8_t cdc_itf_init(void) {
  tx_busy_len = 0;
  tx_out      = tx_in;  // drop what was queued while not configured
  USBD_CDC_SetRxBuffer(&usb_dev, rx_packet);
  return USBD_OK;
}

static int8_t cdc_itf_deinit(void) {
  tx_busy_len = 0;
  return USBD_OK;
}

static int8_t cdc_itf_control(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  switch(cmd) {
    case CDC_SET_LINE_CODING:
      memcpy(line_coding, pbuf, MIN(length, sizeof(line_coding)));
      break;
    case CDC_GET_LINE_CODING:
      memcpy(pbuf, line_coding, MIN(length, sizeof(line_coding)));
      break;
    default:
      break;
  }
  return USBD_OK;
}

static int8_t cdc_itf_receive(uint8_t *buf, uint32_t *len) {
  rb_write(&usb_rx_buf, buf, *len);
  USBD_CDC_SetRxBuffer(&usb_dev, rx_packet);
  USBD_CDC_ReceivePacket(&usb_dev);
  return USBD_OK;
}

static int8_t cdc_itf_transmit_cplt(uint8_t *buf, uint32_t *len, uint8_t epnum) {
  uint32_t out = tx_out + tx_busy_len;
  tx_out       = out >= APP_RX_DATA_SIZE ? 0 : out;
  tx_busy_len  = 0;
  tx_kick();
  return USBD_OK;
}

static USBD_CDC_ItfTypeDef cdc_fops = {
    .Init         = cdc_itf_init,
    .DeInit       = cdc_itf_deinit,
    .Control      = cdc_itf_control,
    .Receive      = cdc_itf_receive,
    .TransmitCplt = cdc_itf_transmit_cplt,
};

void usb_init(void) {
  USBD_Init(&usb_dev, &VCP_Desc, DEVICE_FS);
  USBD_RegisterClass(&usb_dev, &USBD_CDC);
  USBD_CDC_RegisterInterface(&usb_dev, &cdc_fops);
  // USBD_Init leaves D+ detached (SDIS). Keep it off long enough for the host to see a
  // disconnect and enumerate from scratch. Suspected cause of mixed up string
  // descriptors on the first boot after a DFU leave, with only the ~3 ms HAL gap.
  USBD_LL_Delay(20);
  USBD_Start(&usb_dev);
}

uint8_t USB_CDC_is_connected(void) {
  return usb_dev.dev_state == USBD_STATE_CONFIGURED;
}

//TODO: implement new term API
void cdc_init(void) {}

// Free bytes in the IN ring. One byte stays unused: in == out has to mean empty.
static uint32_t cdc_tx_room(void) {
  uint32_t used = (tx_in + APP_RX_DATA_SIZE - tx_out) % APP_RX_DATA_SIZE;
  return APP_RX_DATA_SIZE - 1 - used;
}

// Copy len bytes into the IN ring and publish them with a single store of
// tx_in, so the interrupt never sees a half written packet.
static void cdc_tx_put(const uint8_t *data, uint32_t len) {
  uint32_t in = tx_in;
  while(len--) {
    tx_buf[in] = *data++;
    in         = in + 1 >= APP_RX_DATA_SIZE ? 0 : in + 1;
  }
  __asm__ volatile("" ::: "memory");  // the bytes land before the index moves
  tx_in = in;
}

// Scope packets go whole or not at all: a partial packet desyncs the host, a
// missing one is just a gap.
int cdc_tx(void *data, uint32_t len) {
  if(!cdc_is_connected() || cdc_tx_room() < len) {
    return 0;
  }
  cdc_tx_put((const uint8_t *)data, len);
  NVIC_SetPendingIRQ(OTG_FS_IRQn);  // start sending from the USB interrupt
  return len;
}

// Text waits a little for the interrupt to drain, then drops what does not
// fit rather than overwriting what has not been sent. Without a host it is
// dropped right away.
int cdc_tx_text(const char *data, int len) {
  if(!cdc_is_connected()) {
    return len;  // no host, nothing will drain the ring: drop instead of spinning
  }
  int sent = 0;
  for(uint32_t spin = 0; len > 0 && spin < 200000; spin++) {
    uint32_t room = cdc_tx_room();
    if(room == 0) {
      continue;
    }
    uint32_t n = (uint32_t)len < room ? (uint32_t)len : room;
    cdc_tx_put((const uint8_t *)data + sent, n);
    NVIC_SetPendingIRQ(OTG_FS_IRQn);
    sent += n;
    len -= n;
  }
  return sent;
}

// usb_rx_buf is filled from the USB interrupt (cdc_itf_receive -> rb_write)
// and drained here, and ringbuf.c has no locking: rb_putc's len++ against
// rb_getc's len-- and rb_undo's pos/len rewrite can each lose an update. On
// the bench that merged two commands into "fault0.faultidpmsm0.state" and
// swallowed an enable. Hold the interrupt off for the few dozen bytes this
// touches; anything arriving meanwhile waits in the endpoint.
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
