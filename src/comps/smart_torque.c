// This code were adapted from https://github.com/spietari/stmbl/blob/master/src/comps/smart_torque.c

#include "smart_torque_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

HAL_COMP(smart_torque);

HAL_PIN(error);
HAL_PIN(crc_error);
HAL_PIN(enable);
HAL_PIN(out0);
HAL_PIN(out1);
HAL_PIN(torque_neg);
HAL_PIN(torque_pos);
HAL_PIN(rxpos);
HAL_PIN(debug0);
HAL_PIN(debug1);

struct smart_torque_ctx_t {
  float last_out0;
  uint32_t timer;
};

static volatile uint8_t rxbuf[128];  //rx dma buffer
static volatile uint8_t txbuf[128];  //tx dma buffer
static int rxpos; 

static void sendSerial(uint8_t len) {
  DMA_SetCurrDataCounter(DMA1_Stream4, len);
  DMA_Cmd(DMA1_Stream4, DISABLE);
  DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
  DMA_Cmd(DMA1_Stream4, ENABLE);
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct smart_torque_ctx_t *ctx = (struct smart_torque_ctx_t *)ctx_ptr;
  struct smart_torque_pin_ctx_t *pins = (struct smart_torque_pin_ctx_t *)pin_ptr;
  ctx->last_out0 = -999999;
  ctx->timer = 0;
  PIN(error) = 0;
  PIN(crc_error) = 0;
  PIN(enable) = 0;
  PIN(out0) = 0;
  PIN(out1) = 0;
  PIN(torque_neg) = 0;
  PIN(torque_pos) = 0;
  PIN(rxpos) = 0;
  PIN(debug0) = 123;
  PIN(debug1) = 124;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  DMA_InitTypeDef DMA_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  //USART TX
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  //USART RX
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.USART_BaudRate            = 115200;//2500000;
  USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits            = USART_StopBits_1;
  USART_InitStruct.USART_Parity              = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode                = USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStruct);
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStruct);

  USART_Cmd(USART1, ENABLE);
  USART_Cmd(UART4, ENABLE);

  //RX DMA

  DMA_Cmd(DMA2_Stream5, DISABLE);
  DMA_DeInit(DMA2_Stream5);

  // DMA2-Config
  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&rxbuf;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize         = sizeof(rxbuf);
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream5, &DMA_InitStructure);

  DMA_Cmd(DMA2_Stream5, ENABLE);

  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

  //TX DMA

  DMA_Cmd(DMA1_Stream4, DISABLE);
  DMA_DeInit(DMA1_Stream4);

  // DMA2-Config
  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART4->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&txbuf;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize         = sizeof(txbuf);
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Priority_Low;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);

  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

  //tx enable
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_SetBits(GPIOB, GPIO_Pin_7);

  rxpos = 0;
}

static uint32_t crc32_st(volatile const uint8_t *s, int pos, int buf_len, int expected_payload_len) {
	uint32_t crc = 0xFFFFFFFF;
	for(size_t i = 0; i < expected_payload_len; i++) {
		char ch = s[(pos + i + buf_len) % buf_len];
		for(size_t j = 0; j < 8; j++) {
			uint32_t b = (ch^crc)&1;
			crc >>= 1;
			if(b) crc=crc^0xEDB88320;
			ch>>=1;
		}
	}	
	return ~crc;
}

// Four bytes of data
inline static uint32_t readUInt32_st(volatile uint8_t *buf, int pos) {
  return *(uint32_t*)&buf[pos];
}

// Four bytes of data
inline static float readFloat_st(volatile uint8_t *buf, int pos) {
  return *(float*)&buf[pos];
}


static int isMessageValid_st(volatile uint8_t *buf, int buf_len) {
  if (buf[0] != 0xCA || buf[1] != 0xFE) {
    return 0;
  }
  uint32_t crc_data = crc32_st(buf, 2, buf_len, buf_len - 6);
  uint32_t crc_recv = readUInt32_st(buf, buf_len - 4);  
  return crc_data == crc_recv;
}

static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct smart_torque_ctx_t *ctx = (struct smart_torque_ctx_t *)ctx_ptr;
  struct smart_torque_pin_ctx_t *pins = (struct smart_torque_pin_ctx_t *)pin_ptr;

  float out0 = PIN(out0);
  float out1 = PIN(out1);

  const float EPSILON = 0.01;

  if (ctx->timer >= 1000) {
    ctx->timer = 0;
    if (fabs(ctx->last_out0 - out0) > EPSILON) {

      ctx->last_out0 = out0;

      int index = 0;

      txbuf[index++] = 0xCA;
      txbuf[index++] = 0xFE;

      float *out0_ptr = &out0;
      uint8_t *out0_buf = (uint8_t*)out0_ptr;

      txbuf[index++] = out0_buf[0];
      txbuf[index++] = out0_buf[1];
      txbuf[index++] = out0_buf[2];
      txbuf[index++] = out0_buf[3];

      const int bytes_to_crc = 4;

      uint32_t crc = crc32_st(txbuf, index - bytes_to_crc, 128, bytes_to_crc);
      uint8_t *crc_buf = (uint8_t*)&crc;

      txbuf[index++] = crc_buf[0];
      txbuf[index++] = crc_buf[1];
      txbuf[index++] = crc_buf[2];
      txbuf[index++] = crc_buf[3];

      sendSerial(index);
    }
  }

  ctx->timer++;

  uint32_t bufferpos = sizeof(rxbuf) - DMA_GetCurrDataCounter(DMA2_Stream5);
  //how many packets we have the the rx buffer for processing
  uint32_t available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);

  if (available < 0) {
    return;
  }

  const int expected_payload_length = 28;
  const int expected_msg_length = 2 + expected_payload_length + 4; // 0xCA, 0xFE, payload, 4-byte-CRC

  // Copy rx buffer to a linear buffer
  uint8_t msg_buf[expected_msg_length];
  for (int i = 0; i < expected_msg_length; i++) {
    msg_buf[i] = rxbuf[(rxpos + i) % sizeof(rxbuf)];
  }

  rxpos += available;
  rxpos = rxpos % sizeof(rxbuf);

  PIN(rxpos) = rxpos;

  if (isMessageValid_st(msg_buf, expected_msg_length)) {
    uint32_t enable = readUInt32_st(msg_buf,  2);
    float torque    = readFloat_st (msg_buf, 10);

  //  float in0  = readFloat (rxbuf, rxpos - expected_msg_length + 6,  sizeof(rxbuf));
  //  float in1  = readFloat (rxbuf, rxpos - expected_msg_length + 10, sizeof(rxbuf));

    PIN(enable) = enable == 0xACDC6660;
    PIN(torque_neg) = -torque;
    PIN(torque_pos) =  torque;

    // PIN(torque_neg) = -3;
    // PIN(torque_pos) =  3;

    // PIN(debug1) = temp;
    // PIN(in0) = in0;
    // PIN(in1) = in1;
  }
}

const hal_comp_t smart_torque_comp_struct = {
    .name      = "smart_torque",
    .nrt       = 0,
    .rt        = 0,
    .frt       = frt_func,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct smart_torque_ctx_t),
    .pin_count = sizeof(struct smart_torque_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};