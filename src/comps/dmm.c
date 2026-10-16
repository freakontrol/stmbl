#include "comps/dmm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"

/**
* ## Brief
* `dmm` reads the absolute position of a DMM servo motor encoder that streams its angle continuously over RS485 into the fb0 connector of the F4 logic board. conf/template/encdmm_fb0.txt loads it and wires `fb_switch0.mot_pos = dmm0.pos`, `fb_switch0.mot_abs_pos = dmm0.pos`, `fb_switch0.mot_state = 3`, `fault0.mot_fb_error = dmm0.error` and `io0.fb0y = dmm0.error`.
*
* ## Component Explanation
*
* 1. **Receiver (hw_init)**:
* - USART6 in half-duplex mode on PC6 at 1.875 Mbaud 8N1, the fb0 transmitter (PD15) is switched off, so the drive only listens. DMA2 stream 1 writes the incoming bytes into a 15 byte circular buffer.
*
* 2. **Frame format**:
* - A frame has 4 bytes. Byte 0 holds the MSBs of the other three bytes (bits 0..2), 4 bits F (unused here) and a 0 in bit 7; bytes 1..3 carry 7 data bits and a 1 in bit 7.
* - Byte 1 + MSB = angle high byte, byte 2 + MSB = angle low byte, byte 3 + MSB = CRC-8 (polynomial x^8 + x^3 + 1) over the 16 bit angle.
*
* 3. **Decoding (rt)**:
* - From the current DMA write position, the component looks for a valid frame starting 7, 8, 9 or 10 bytes back (the newest 3 bytes are skipped because they may belong to an incomplete frame). A frame is valid if the CRC and the four framing bits match.
* - On success `pos = angle * 2 * pi / 65536 - pi` (rad, one turn = 16 bit) and `error = 0`; if no valid frame is found `error = 1` and `pos` keeps its last value.
*
* {{% hint warning %}}
* The receive buffer is never cleared (see the TODO in `rt`). If the encoder stops sending, the last valid frame stays in the buffer and `error` stays 0 with a frozen `pos`, so a cable break is not detected. `dmm` and `usart` both use USART6 and DMA2 stream 1 and cannot be loaded together.
* {{% /hint %}}
*/

HAL_COMP(dmm);

HAL_PIN(pos);   // *output*, absolute encoder position (rad), -pi..pi
HAL_PIN(error); // *output*, 1 = no valid frame found this rt period, 0 = ok
HAL_PIN(state); // *output*, unused
HAL_PIN(dma);   // *output*, dma transfers left, DMA write position in the ring buffer
HAL_PIN(dump);  // *input*, unused

#pragma pack(push, 1)
typedef struct DMM_AsBitFieldInts {
  // byte 0
  unsigned int C7 : 1;
  unsigned int L7 : 1;
  unsigned int H7 : 1;
  unsigned int F : 4;
  unsigned int _byte0_x : 1;

  // byte 1
  unsigned int H : 7;
  unsigned int _byte1_1 : 1;

  // byte 2
  unsigned int L : 7;
  unsigned int _byte2_1 : 1;

  // byte 3
  unsigned int C : 7;
  unsigned int _byte3_1 : 1;
} DMM_AsBitFieldInts;
typedef struct DMM_AsBitFieldBits {
  // byte 0
  unsigned int C7 : 1;
  unsigned int L7 : 1;
  unsigned int H7 : 1;
  unsigned int F0 : 1;
  unsigned int F1 : 1;
  unsigned int F2 : 1;
  unsigned int F3 : 1;
  unsigned int _byte0_x : 1;

  // byte 1
  unsigned int H0 : 1;
  unsigned int H1 : 1;
  unsigned int H2 : 1;
  unsigned int H3 : 1;
  unsigned int H4 : 1;
  unsigned int H5 : 1;
  unsigned int H6 : 1;
  unsigned int _byte1_1 : 1;

  // byte 2
  unsigned int L0 : 1;
  unsigned int L1 : 1;
  unsigned int L2 : 1;
  unsigned int L3 : 1;
  unsigned int L4 : 1;
  unsigned int L5 : 1;
  unsigned int L6 : 1;
  unsigned int _byte2_1 : 1;

  // byte 3
  unsigned int C0 : 1;
  unsigned int C1 : 1;
  unsigned int C2 : 1;
  unsigned int C3 : 1;
  unsigned int C4 : 1;
  unsigned int C5 : 1;
  unsigned int C6 : 1;
  unsigned int _byte3_1 : 1;
} DMM_AsBitFieldBits;
typedef union DMM_Packet {
  uint8_t bytes[4];
  DMM_AsBitFieldInts ints;
  DMM_AsBitFieldBits bits;
} DMM_Packet;
#pragma pack(pop)

uint8_t SGenerate8BitsCRC(int16_t Data16) /*Input 16bits Data, make 8 bits CRC for X^8 + X^3 + 1    */
{
  int16_t StateX, StateY, i;
  uint8_t CRC8bits;
  StateX = Data16 & 0xFF00;  //High 8bits for the States
  Data16 = Data16 << 8;
  for(i = 0; i < 8; i++) {
    StateY = StateX << 1;
    if((StateX & 0x8000) != 0) {  //X7 = 1
      if((Data16 & 0x8000) != 0)
        StateY = StateY & 0xFE00;  //D15=1 so X0=0
      else
        StateY = StateY | 0x0100;  //D15=0 so X0=1
      if((StateX & 0x0400) != 0)
        StateY = StateY & 0xF700;  //X2 = 1 so X3 = 0
      else
        StateY = StateY | 0x0800;  //X2 = 0 so X3 = 1
    } else {                       //X7 = 0
      if((Data16 & 0x8000) != 0)
        StateY = StateY | 0x0100;  //D15=1 so X0=1
      else
        StateY = StateY & 0xFE00;  //D15=0 so X0=0
      if((StateX & 0x0400) != 0)
        StateY = StateY | 0x0800;  //X2 = 1 so X3 = 1
      else
        StateY = StateY & 0xF700;  //X2 = 0 so X3 = 0
    }
    StateX = StateY;
    Data16 = Data16 << 1;
  }
  StateX   = StateX >> 8;
  CRC8bits = StateX & 0x00FF;
  return (CRC8bits);
}

struct dmm_ctx_t {
  uint8_t rxbuf[15];
  DMM_Packet data;
};

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dmm_ctx_t *ctx = (struct dmm_ctx_t *)ctx_ptr;
  // struct dmm_pin_ctx_t * pins = (struct dmm_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
  LL_USART_InitTypeDef USART_InitStruct;
  LL_USART_StructInit(&USART_InitStruct);
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);

  //TX enable
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

  //USART TX
  GPIO_InitStruct.Pin   = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //USART RX
  // gpio_set_af(GPIOB, 11, LL_GPIO_AF_8);
  // GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  // gpio_init(GPIOB, &GPIO_InitStruct);

  USART_InitStruct.BaudRate            = 1875000;
  USART_InitStruct.DataWidth          = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection                = LL_USART_DIRECTION_TX_RX;
  LL_USART_Init(USART6, &USART_InitStruct);

  /* Enable the USART */
  LL_USART_Enable(USART6);
  SET_BIT(USART6->CR3, USART_CR3_HDSEL);  // half duplex

  // DMA-Disable
  dma_stream_stop(DMA2_Stream1);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_1);

  // DMA2-Config
  DMA_InitStructure.Channel            = LL_DMA_CHANNEL_5;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (USART6->DR);
  DMA_InitStructure.MemoryOrM2MDstAddress    = (uint32_t)&ctx->rxbuf;
  DMA_InitStructure.Direction                = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.NbData         = ARRAY_SIZE(ctx->rxbuf);
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
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &DMA_InitStructure);
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx disable
  LL_USART_EnableDMAReq_RX(USART6);
  LL_DMA_ClearFlag_TC1(DMA2);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dmm_ctx_t *ctx      = (struct dmm_ctx_t *)ctx_ptr;
  struct dmm_pin_ctx_t *pins = (struct dmm_pin_ctx_t *)pin_ptr;
  PIN(dma)                   = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);
  ;

  //next received byte will be written to bufferpos
  uint32_t bufferpos = ARRAY_SIZE(ctx->rxbuf) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);

  uint8_t crc    = 0;
  uint16_t angle = 0;
  uint8_t found  = 0;
  //TODO: clear buffer after reading
  for(uint32_t i = 4; i < 8; i++) {
    for(int j = 0; j < 4; j++) {
      ctx->data.bytes[j] = ctx->rxbuf[(bufferpos + ARRAY_SIZE(ctx->rxbuf) - (i + 3 - j)) % ARRAY_SIZE(ctx->rxbuf)];
    }
    crc         = (ctx->data.ints.C7 << 7) | ctx->data.ints.C;
    angle       = (ctx->data.ints.H7 << 15) | (ctx->data.ints.H << 8) | (ctx->data.ints.L7 << 7) | ctx->data.ints.L;
    uint8_t res = SGenerate8BitsCRC(angle);
    if(res == crc && ctx->data.bits._byte1_1 == 1 && ctx->data.bits._byte2_1 == 1 && ctx->data.bits._byte3_1 == 1 && ctx->data.bits._byte0_x == 0) {
      found = 1;
      break;
    }
  }
  if(found) {
    PIN(pos)   = (angle * M_PI * 2.0f / 65536.0f) - M_PI;
    PIN(error) = 0;
  } else {
    PIN(error) = 1;
  }
}

hal_comp_t dmm_comp_struct = {
    .name      = "dmm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct dmm_ctx_t),
    .pin_count = sizeof(struct dmm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
