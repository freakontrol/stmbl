#pragma once

#include <stdint.h>
#include "ringbuf.h"

#define APP_TX_BUF_SIZE 128
#define APP_RX_BUF_SIZE 128


extern struct ringbuf usb_rx_buf;
extern struct ringbuf usb_tx_buf;

uint8_t USB_CDC_is_connected(void);
void usb_init(void);

void cdc_init(void);
int cdc_tx(void *data, uint32_t len);
int cdc_tx_text(const char *data, int len);
int cdc_getline(char *ptr, int len);
int cdc_is_connected();
void cdc_poll();
