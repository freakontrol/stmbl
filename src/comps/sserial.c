#include "sserial_comp.h"
/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2015 Ian McMahon <facetious.ian@gmail.com>
* Copyright (C) 2013 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"
#include "sserial.h"
#include "crc8.h"
#include "setup.h"
#include <string.h>

HAL_COMP(sserial);

// pins
HAL_PIN(error);
HAL_PIN(crc_error);  //counts crc errors, is never reset
HAL_PIN(connected);  //connection status TODO: not stable during startup, needs link to pd
HAL_PIN(timeout);    // 20khz / 1khz * 2 reads = 40

HAL_PIN(pos_cmd);
HAL_PIN(pos_cmd_d);
HAL_PIN(pos_fb);
HAL_PIN(vel_fb);
HAL_PIN(current);
HAL_PIN(scale);

HAL_PIN(clock_scale);
HAL_PIN(available);
HAL_PIN(phase);

HAL_PIN(in0);
HAL_PIN(in1);
HAL_PIN(in2);
HAL_PIN(in3);
HAL_PIN(fault);
HAL_PIN(fault_code);

HAL_PIN(out0);
HAL_PIN(out1);
HAL_PIN(out2);
HAL_PIN(out3);
HAL_PIN(enable);
HAL_PIN(index_clear);
HAL_PIN(index_out);
HAL_PIN(pos_advance);
HAL_PIN(pos_extrap);  // 1: pos_fb extrapolated by vel_fb to the reply; 0 for a vel only feedback (sserial_uf)

//TODO: move to ctx
struct sserial_ctx_t {
  uint32_t phase;
};

static volatile uint8_t rxbuf[128];  //rx dma buffer
static volatile uint8_t txbuf[128];  //tx dma buffer
static uint16_t address;             //current address pointer
static int rxpos;                    //read pointer for rx ringbuffer
static uint32_t timeout;
static uint32_t crc_bad;  // process data packets in a row with a bad crc
// bad packets in a row that keep the last good command; one more drops it
#define CRC_BAD_HOLD 2
static lbp_t lbp;
static const char name[] = LBPCardName;
static unit_no_t unit;
static uint32_t max_waste_ticks;
static uint32_t block_bytes;

#pragma pack(push, 1)
//*****************************************************************************
uint8_t sserial_slave[] = {
    0x0C,
    0x0C,
    0x8B,
    0x01,
    0xA9,
    0x01,
    0x00,
    0x00,  // 0..7
    0x00,
    0x00,
    0x00,
    0x00,
    0xA0,
    0x20,
    0x10,
    0x80,  // 8..15
    0x00,
    0x00,
    0x80,
    0xFF,
    0x00,
    0x00,
    0x80,
    0x7F,  // 16..23
    0x08,
    0x00,
    0x72,
    0x61,
    0x64,
    0x00,
    0x70,
    0x6F,  // 24..31
    0x73,
    0x5F,
    0x63,
    0x6D,
    0x64,
    0x00,
    0x00,
    0x00,  // 32..39
    0x00,
    0x00,
    0x00,
    0x00,
    0xA0,
    0x20,
    0x10,
    0x80,  // 40..47
    0x00,
    0x00,
    0x80,
    0xFF,
    0x00,
    0x00,
    0x80,
    0x7F,  // 48..55
    0x26,
    0x00,
    0x72,
    0x61,
    0x64,
    0x00,
    0x76,
    0x65,  // 56..63
    0x6C,
    0x5F,
    0x63,
    0x6D,
    0x64,
    0x00,
    0x00,
    0x00,  // 64..71
    0xA0,
    0x04,
    0x01,
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,  // 72..79
    0x00,
    0x00,
    0x80,
    0x3F,
    0x46,
    0x00,
    0x6E,
    0x6F,  // 80..87
    0x6E,
    0x65,
    0x00,
    0x6F,
    0x75,
    0x74,
    0x00,
    0x00,  // 88..95
    0xA0,
    0x01,
    0x07,
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,  // 96..103
    0x00,
    0x00,
    0x80,
    0x3F,
    0x5F,
    0x00,
    0x6E,
    0x6F,  // 104..111
    0x6E,
    0x65,
    0x00,
    0x65,
    0x6E,
    0x61,
    0x62,
    0x6C,  // 112..119
    0x65,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 120..127
    0xA0,
    0x20,
    0x10,
    0x00,
    0x00,
    0x00,
    0x80,
    0xFF,  // 128..135
    0x00,
    0x00,
    0x80,
    0x7F,
    0x7A,
    0x00,
    0x72,
    0x61,  // 136..143
    0x64,
    0x00,
    0x70,
    0x6F,
    0x73,
    0x5F,
    0x66,
    0x62,  // 144..151
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 152..159
    0xA0,
    0x20,
    0x10,
    0x00,
    0x00,
    0x00,
    0x80,
    0xFF,  // 160..167
    0x00,
    0x00,
    0x80,
    0x7F,
    0x99,
    0x00,
    0x72,
    0x61,  // 168..175
    0x64,
    0x00,
    0x76,
    0x65,
    0x6C,
    0x5F,
    0x66,
    0x62,  // 176..183
    0x00,
    0x00,
    0x00,
    0x00,
    0xA0,
    0x08,
    0x03,
    0x00,  // 184..191
    0x00,
    0x00,
    0xF0,
    0xC1,
    0x00,
    0x00,
    0xF0,
    0x41,  // 192..199
    0xB9,
    0x00,
    0x41,
    0x00,
    0x63,
    0x75,
    0x72,
    0x72,  // 200..207
    0x65,
    0x6E,
    0x74,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 208..215
    0xA0,
    0x04,
    0x01,
    0x00,
    0x00,
    0x00,
    0xC8,
    0xC2,  // 216..223
    0x00,
    0x00,
    0xC8,
    0x42,
    0xD4,
    0x00,
    0x6E,
    0x6F,  // 224..231
    0x6E,
    0x65,
    0x00,
    0x69,
    0x6E,
    0x00,
    0x00,
    0x00,  // 232..239
    0xA0,
    0x01,
    0x07,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 240..247
    0x00,
    0x00,
    0x80,
    0x3F,
    0xEE,
    0x00,
    0x6E,
    0x6F,  // 248..255
    0x6E,
    0x65,
    0x00,
    0x66,
    0x61,
    0x75,
    0x6C,
    0x74,  // 256..263
    0x00,
    0x00,
    0x00,
    0x00,
    0xA0,
    0x01,
    0x07,
    0x40,  // 264..271
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x80,
    0x3F,  // 272..279
    0x09,
    0x01,
    0x6E,
    0x6F,
    0x6E,
    0x65,
    0x00,
    0x69,  // 280..287
    0x6E,
    0x64,
    0x65,
    0x78,
    0x5F,
    0x65,
    0x6E,
    0x61,  // 288..295
    0x62,
    0x6C,
    0x65,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 296..303
    0xA0,
    0x20,
    0x10,
    0x80,
    0x00,
    0x00,
    0x80,
    0xFF,  // 304..311
    0x00,
    0x00,
    0x80,
    0x7F,
    0x2C,
    0x01,
    0x6E,
    0x6F,  // 312..319
    0x6E,
    0x65,
    0x00,
    0x73,
    0x63,
    0x61,
    0x6C,
    0x65,  // 320..327
    0x00,
    0xB0,
    0x00,
    0x01,
    0x00,
    0x50,
    0x6F,
    0x73,  // 328..335
    0x69,
    0x74,
    0x69,
    0x6F,
    0x6E,
    0x20,
    0x6D,
    0x6F,  // 336..343
    0x64,
    0x65,
    0x00,
    0x00,
    0xA0,
    0x02,
    0x00,
    0x00,  // 344..351
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 352..359
    0x5B,
    0x01,
    0x00,
    0x70,
    0x61,
    0x64,
    0x64,
    0x69,  // 360..367
    0x6E,
    0x67,
    0x00,
    0x00,
    0xA0,
    0x02,
    0x00,
    0x80,  // 368..375
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 376..383
    0x73,
    0x01,
    0x00,
    0x70,
    0x61,
    0x64,
    0x64,
    0x69,  // 384..391
    0x6E,
    0x67,
    0x00,
    0x0C,
    0x00,
    0x2C,
    0x00,
    0x48,  // 392..399
    0x00,
    0x60,
    0x00,
    0x80,
    0x00,
    0xA0,
    0x00,
    0xBC,  // 400..407
    0x00,
    0xD8,
    0x00,
    0xF0,
    0x00,
    0x0C,
    0x01,
    0x5C,  // 408..415
    0x01,
    0x74,
    0x01,
    0xAF,
    0x01,
    0xCD,
    0x01,
    0x00,  // 416..423
    0x00,
    0x49,
    0x01,
    0x00,
    0x00,
    0x00,
    0x00,
    0xA0,  // 424..431
    0x08,
    0x02,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,  // 432..439
    0x00,
    0x7F,
    0x43,
    0xAF,
    0x01,
    0x6E,
    0x6F,
    0x6E,  // 440..447
    0x65,
    0x00,
    0x66,
    0x61,
    0x75,
    0x6C,
    0x74,
    0x5F,  // 448..455
    0x63,
    0x6F,
    0x64,
    0x65,
    0x00,
    0xA0,
    0x18,
    0x02,  // 456..463
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,
    0x16,
    0xB7,
    0xD1,  // 464..471
    0x44,
    0xCD,
    0x01,
    0x6E,
    0x6F,
    0x6E,
    0x65,
    0x00,  // 472..479
    0x73,
    0x63,
    0x61,
    0x6C,
    0x65,
    0x00,
};

const discovery_rpc_t discovery = {
    .ptocp  = 0x018B,
    .gtocp  = 0x01A9,
    .input  = 12,
    .output = 12,
};

typedef struct {
  float pos_cmd;
  float vel_cmd;
  uint32_t out_0 : 1;
  uint32_t out_1 : 1;
  uint32_t out_2 : 1;
  uint32_t out_3 : 1;
  uint32_t enable : 1;
  uint32_t index_enable : 1;
  uint32_t padding : 2;
  // linrev scale from LinuxCNC every packet, 0 = keep the current scale.
  // 24 bit unsigned, 1e-4 per count: hm2's standard sserial module carries
  // 96 bits per direction (3 registers), so a float does not fit. hm2
  // writes (u64)(val / ParmMax * (2^24 - 1)); ParmMax 1677.7214f sits
  // just below 2^24 * 1e-4, so the truncation lands on val * 1e4 exactly
  // for every multiple of 1e-4 (checked for all 2^24 counts on the host).
  uint8_t scale[3];
} sserial_out_process_data_t;  //size:12 bytes
_Static_assert(sizeof(sserial_out_process_data_t) == 12, "sserial_out_process_data_t size error!");

typedef struct {
  float pos_fb;
  float vel_fb;
  int8_t current;
  uint32_t in_0 : 1;
  uint32_t in_1 : 1;
  uint32_t in_2 : 1;
  uint32_t in_3 : 1;
  uint32_t fault : 1;
  uint32_t index_enable : 1;
  uint32_t padding : 2;
  uint8_t fault_code;  // fault0.last_fault while fault is set, else 0
} sserial_in_process_data_t;  //size:11 bytes
_Static_assert(sizeof(sserial_in_process_data_t) == 11, "sserial_in_process_data_t size error!");
//******************************************************************************
#pragma pack(pop)

static sserial_out_process_data_t data_out;
static sserial_in_process_data_t data_in;

static uint8_t crc_reuest(uint8_t len) {
  uint8_t crc = crc8_init();
  for(int i = rxpos; i < rxpos + len; i++) {
    crc = crc8_update(crc, (void *)&(rxbuf[i % sizeof(rxbuf)]), 1);
  }
  crc = crc8_finalize(crc);
  return crc == rxbuf[(rxpos + len) % sizeof(rxbuf)];
}

static uint8_t crc8(uint8_t *addr, uint8_t len) {
  uint8_t crc = crc8_init();
  crc         = crc8_update(crc, addr, len);
  return crc8_finalize(crc);
}

static void send(uint8_t len, uint8_t docrc) {
  timeout = 0;
  // NDTR only takes a write while the stream is off (RM0090 10.5.6)
  dma_stream_stop(DMA1_Stream4);
  if(docrc) {
    txbuf[len] = crc8((uint8_t *)txbuf, len);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, len + 1);
  } else {
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, len);
  }
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
}

//v3
//pb13 txen
//pc12 usart5 tx
//pa9 usart1 tx as rx
//USART5 TX DMA1 stream7 channel4
//USART1 RX DMA2 stream5 channel4

//v4.1
//pa0 usart4 tx DMA1 stream4 channel4 CMD_36
//pa10 usart1 rx DMA2 stream5 channel4 CMD_12

//TODO: lbp command 0xe6 to set mode

// at load, before the template's pin settings (hw_init runs after them)
static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sserial_pin_ctx_t *pins = (struct sserial_pin_ctx_t *)pin_ptr;
  PIN(pos_extrap)                = 1.0;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct sserial_ctx_t * ctx = (struct sserial_ctx_t *)ctx_ptr;
  struct sserial_pin_ctx_t *pins = (struct sserial_pin_ctx_t *)pin_ptr;

  PIN(timeout) = 100.0;  // 20khz / 1khz * 2 reads = 40

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_GPIO_StructInit(&GPIO_InitStruct);
  LL_USART_InitTypeDef USART_InitStruct;
  LL_USART_StructInit(&USART_InitStruct);
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
  //USART TX
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //USART RX
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.BaudRate            = 2500000;
  USART_InitStruct.DataWidth          = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection                = LL_USART_DIRECTION_RX;
  LL_USART_Init(USART1, &USART_InitStruct);
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  LL_USART_Init(UART4, &USART_InitStruct);

  LL_USART_Enable(USART1);
  LL_USART_Enable(UART4);

  //RX DMA

  dma_stream_stop(DMA2_Stream5);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_5);

  // DMA2-Config
  DMA_InitStructure.Channel            = LL_DMA_CHANNEL_4;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (USART1->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress    = (uint32_t)&rxbuf;
  DMA_InitStructure.Direction                = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.NbData         = sizeof(rxbuf);
  DMA_InitStructure.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode               = LL_DMA_MODE_CIRCULAR;
  DMA_InitStructure.Priority           = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst    = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA2, LL_DMA_STREAM_5, &DMA_InitStructure);

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);

  LL_USART_EnableDMAReq_RX(USART1);

  //TX DMA

  dma_stream_stop(DMA1_Stream4);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_4);

  // DMA2-Config
  DMA_InitStructure.Channel            = LL_DMA_CHANNEL_4;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (UART4->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress    = (uint32_t)&txbuf;
  DMA_InitStructure.Direction                = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData         = sizeof(txbuf);
  DMA_InitStructure.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode               = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority           = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst    = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA1, LL_DMA_STREAM_4, &DMA_InitStructure);

  LL_USART_EnableDMAReq_TX(UART4);

  //tx enable
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);

  //generate unit number from 96bit unique chip ID
  unit.unit = U_ID[0] ^ U_ID[1] ^ U_ID[2];

  rxpos   = 0;
  timeout = 1000;  //make sure we start in timeout

  //bytes to wait before expected end of transmission to prevent timeouts
  block_bytes = 5;
  //calculate timeout in systicks for block_bytes
  max_waste_ticks = (1.0 / 2500000.0) * 11.0 * (float)block_bytes / (1.0f / (float)hal_get_systick_freq());

  PIN(clock_scale) = 1.0;
  PIN(phase)       = 0;
}

// pos_fb is computed once per rt period, but the reply goes out whenever
// LinuxCNC's packet arrives, 0 to 200 us later, so a moving axis reported
// a position up to one period old (+-0.04 one-sample f-error spikes at
// 170/s on Y). The rt (after linrev, see rt_prio in sserial.txt) keeps
// pos_fb, vel_fb and its period; the reply extrapolates by the time since
// that period started. Two slots, swapped by one word write, as the frt
// can preempt the rt halfway through a store.
extern volatile uint32_t rt_tick;  // main.c, DMA2_Stream0_IRQHandler
static struct {
  float pos;
  float vel;
  uint32_t tick;
} fb_snap[2];
static volatile uint32_t fb_snap_i;

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sserial_pin_ctx_t *pins = (struct sserial_pin_ctx_t *)pin_ptr;
  uint32_t i     = fb_snap_i ^ 1;
  fb_snap[i].pos  = PIN(pos_fb);
  fb_snap[i].vel  = PIN(vel_fb);
  fb_snap[i].tick = rt_tick;
  fb_snap_i       = i;
}

// time since the rt period of the last snapshot started [s]
static float fb_age(uint32_t tick) {
  // the adc dma restarts its count at every rt period: its progress is the
  // time into the current one
  // the period may end (TC set, count reloaded) between the reads: read the
  // tick and flag on both sides of the count and retry if they differ, else
  // the old period's count lands on the new period's tick, one period too old
  uint32_t ticks, ndtr, flag;
  do {
    ticks = rt_tick;
    flag  = LL_DMA_IsActiveFlag_TC0(DMA2);
    ndtr  = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0);
  } while(ticks != rt_tick || flag != LL_DMA_IsActiveFlag_TC0(DMA2));
  if(flag) {  // a period started, its irq not yet in
    ticks++;
  }
  float age = (float)(ticks - tick) + (float)(ADC_SAMPLES_IN_RT - ndtr) / (float)ADC_SAMPLES_IN_RT;
  return CLAMP(age, 0.0, 2.0) / (float)RT_FREQ;
}

static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct sserial_pin_ctx_t *pins = (struct sserial_pin_ctx_t *)pin_ptr;
  //struct sserial_ctx_t *mem = (struct sserial_ctx_t *)ctx_ptr;
  //next received packet will be written to bufferpos
  uint32_t bufferpos = sizeof(rxbuf) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5);
  //how many packets we have the the rx buffer for processing
  uint32_t available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);

  PIN(phase) += 1.0;

  uint32_t goal    = 5;
  PIN(clock_scale) = 1.0;
  if(PIN(phase) > 3) {
    PIN(phase) = 0;
    if(available > goal) {
      PIN(clock_scale) = 0.9;
    } else if(available < goal && available > 0) {
      PIN(clock_scale) = 1.1;
    }
  }

  PIN(available) = available;

  if(available >= 1) {
    lbp.byte = rxbuf[rxpos];

    if(lbp.ct == CT_LOCAL && lbp.wr == 0) {  //local read, cmd+crc = 2b
      timeout = 0;
      if(available >= 2) {
        switch(lbp.byte) {
          case LBPCookieCMD:
            txbuf[0] = LBPCookie;
            break;
          case LBPStatusCMD:  //TODO: return status
            txbuf[0] = 0x00;
            break;
          case LBPCardName0Cmd ... LBPCardName3Cmd:
            txbuf[0] = name[lbp.byte - LBPCardName0Cmd];
            break;
          default:  //TODO: handle unknown command condition
            txbuf[0] = 0x00;
        }
        send(1, 1);
        rxpos += 2;
      }
    } else if(lbp.ct == CT_LOCAL && lbp.wr == 1) {  //local write, cmd+data+crc = 3b
      timeout = 0;
      //0xFF and 0xFC are not followed by crc
      if(rxbuf[rxpos] == 0xFF) {
        // reset parser
        rxpos += 1;
      } else if(rxbuf[rxpos] == 0xFC) {
        // todo
        rxpos += 1;
      } else if(available >= 3) {  //writes do not expect crc in reply
        txbuf[0] = 0x00;
        send(1, 0);
        rxpos += 3;
      }
    } else if(lbp.ct == CT_RPC) {  //RPC TODO: check for ct should not required for rpc
      timeout = 0;
      if(lbp.byte == UnitNumberRPC && available >= 2) {  //unit number, cmd+crc = 2b
        txbuf[0] = unit.byte[0];
        txbuf[1] = unit.byte[1];
        txbuf[2] = unit.byte[2];
        txbuf[3] = unit.byte[3];
        send(4, 1);
        rxpos += 2;
      } else if(lbp.byte == DiscoveryRPC && available >= 2) {  //discovery, cmd+crc = 2b
        memcpy((void *)txbuf, ((uint8_t *)&discovery), sizeof(discovery));
        send(sizeof(discovery), 1);
        rxpos += 2;
      } else if(lbp.byte == ProcessDataRPC && available >= discovery.output + 2 - block_bytes) {  //process data, requires cmd+output bytes+crc
        uint32_t t1         = hal_get_systick_value();
        uint32_t wait_ticks = 0;
        //wait with timeout until rest of process data is received
        do {
          uint32_t t2 = hal_get_systick_value();
          if(t1 < t2) {
            t1 += hal_get_systick_reload();
          }
          wait_ticks = t1 - t2;
          //next received packet will be written to bufferpos
          bufferpos = sizeof(rxbuf) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5);
          //how many packets we have the the rx buffer for processing
          available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);
        } while(available < discovery.output + 2 && wait_ticks <= max_waste_ticks);
        //TODO: fault handling on timeout...
        //set input pins
        uint32_t si     = fb_snap_i;
        data_in.pos_fb  = fb_snap[si].pos + (PIN(pos_extrap) > 0.0 ? fb_snap[si].vel * (PIN(pos_advance) + fb_age(fb_snap[si].tick)) : 0.0);
        data_in.vel_fb  = fb_snap[si].vel;
        data_in.current = CLAMP(PIN(current) / (30.0f / 128.0f), -127, 127);
        data_in.in_0    = (PIN(in0) > 0) ? 1 : 0;
        data_in.in_1    = (PIN(in1) > 0) ? 1 : 0;
        data_in.in_2    = (PIN(in2) > 0) ? 1 : 0;
        data_in.in_3    = (PIN(in3) > 0) ? 1 : 0;
        data_in.fault   = (PIN(fault) > 0) ? 1 : 0;
        data_in.fault_code = data_in.fault ? CLAMP(PIN(fault_code), 0, 255) : 0;  // last_fault outlives the fault

        //copy output pins from rx buffer
        for(int i = 0; i < discovery.output; i++) {
          ((uint8_t *)(&data_out))[i] = rxbuf[(rxpos + i + 1) % sizeof(rxbuf)];
        }

        //set bidirectional pins
        PIN(index_out)       = data_out.index_enable;
        data_in.index_enable = (PIN(index_clear) > 0) ? 0 : data_out.index_enable;

        //copy input pins to tx buffer
        txbuf[0] = 0x00;  //fault byte
        for(int i = 0; i < (discovery.input - 1); i++) {
          txbuf[i + 1] = ((uint8_t *)(&data_in))[i];
        }
        if(crc_reuest(discovery.output + 1)) {
          //send buffer
          txbuf[discovery.input] = crc8((uint8_t *)txbuf, discovery.input);
          dma_stream_stop(DMA1_Stream4);
          LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, discovery.input + 1);
          LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
          //send(discovery.input, 1);
          timeout = 0;
          crc_bad = 0;
          //set output pins

          PIN(pos_cmd)   = data_out.pos_cmd;
          PIN(pos_cmd_d) = data_out.vel_cmd;
          PIN(out0)      = data_out.out_0;
          PIN(out1)      = data_out.out_1;
          PIN(out2)      = data_out.out_2;
          PIN(out3)      = data_out.out_3;
          PIN(enable)    = data_out.enable;
          // in process data: a global only reaches the drive before process
          // data starts, and hm2 2.9's change check skips round values
          uint32_t scale_raw = data_out.scale[0] | (data_out.scale[1] << 8) | (data_out.scale[2] << 16);
          if(scale_raw) {
            PIN(scale) = scale_raw / 10000.0f;  // divide: 10 comes out as exactly 10
          }
        } else {
          // bad packet: count it and keep the last good command for up to
          // CRC_BAD_HOLD packets. Zeroing enable on every one dropped
          // fault0.en on a single bad packet. The RPC header resets the
          // timeout, so a run of bad packets is caught here, not there.
          PIN(crc_error)
          ++;
          if(++crc_bad > CRC_BAD_HOLD) {
            PIN(pos_cmd)   = 0;
            PIN(pos_cmd_d) = 0;
            PIN(out0)      = 0;
            PIN(out1)      = 0;
            PIN(out2)      = 0;
            PIN(out3)      = 0;
            PIN(enable)    = 0;
          }
        }
        rxpos += discovery.output + 2;
      }
    } else if(lbp.ct == CT_RW && lbp.wr == 0) {  //read
      //size = 1 + 2*lbp.as  + 1
      int size = 2 * lbp.as + 2;
      timeout  = 0;
      if(available >= size) {
        if(lbp.as) {  //address included in command = cmd+addr+addr+crc
          address = rxbuf[(rxpos + 1) % sizeof(rxbuf)] + (rxbuf[(rxpos + 2) % sizeof(rxbuf)] << 8);
          rxpos += 4;
        } else {  //address not included in command = cmd+crc
          rxpos += 2;
        }
        //an address past the table reads zeros: the host still gets its reply
        if((address + (1 << lbp.ds)) <= ARRAY_SIZE(sserial_slave)) {  //check if address is valid
          memcpy((void *)txbuf, &sserial_slave[address], (1 << lbp.ds));
        } else {
          memset((void *)txbuf, 0, (1 << lbp.ds));
        }
        send((1 << lbp.ds), 1);
        if(lbp.ai) {  //auto increment address by datasize
          address += (1 << lbp.ds);
        }
      }
    } else if(lbp.ct == CT_RW && lbp.wr == 1) {  // lbp (addr1 addr2) data0, data1,...
      //size = 1 + 2*ai +ds +crc
      int size = 2 * lbp.as + (1 << lbp.ds) + 2;
      timeout  = 0;
      if(available >= size) {
        if(lbp.as) {  //address included in command = cmd+addr+addr+crc
          address = rxbuf[(rxpos + 1) % sizeof(rxbuf)] + (rxbuf[(rxpos + 2) % sizeof(rxbuf)] << 8);
          rxpos += 3;
        } else {  //address not included in command = cmd+crc
          rxpos += 1;
        }
        if((address + (1 << lbp.ds)) <= ARRAY_SIZE(sserial_slave)) {  //check if address is valid
          for(int i = 0; i < (1 << lbp.ds); i++) {
            sserial_slave[address + i] = rxbuf[(rxpos + i) % sizeof(rxbuf)];
          }
        }
        rxpos += (1 << lbp.ds) + 1;
        if(lbp.ai) {  //auto increment address by datasize
          address += (1 << lbp.ds);
        }
      }
    } else {
      //TODO: handle unkown packet
    }
  }

  if(timeout > PIN(timeout)) {  //TODO: clamping
    PIN(connected) = 0;
    PIN(error)     = 1;
    PIN(pos_cmd)   = 0;
    PIN(pos_cmd_d) = 0;
    PIN(out0)      = 0;
    PIN(out1)      = 0;
    PIN(out2)      = 0;
    PIN(out3)      = 0;
    PIN(enable)    = 0;
    rxpos          = bufferpos;
  } else if(crc_bad > CRC_BAD_HOLD) {
    PIN(connected) = 0;
    PIN(error)     = 1;
  } else {
    PIN(connected) = 1;
    PIN(error)     = 0;
  }
  rxpos = rxpos % sizeof(rxbuf);
  timeout++;
}

const hal_comp_t sserial_comp_struct = {
    .name      = "sserial",
    .nrt       = 0,  //nrt_func,
    .rt        = rt_func,
    .frt       = frt_func,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct sserial_ctx_t),
    .pin_count = sizeof(struct sserial_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
