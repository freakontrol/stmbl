// This code were adapted from https://github.com/spietari/stmbl/blob/master/src/comps/smart_torque.c

#include "smart_torque_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
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
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, len);
  dma_stream_stop(DMA1_Stream4);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
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

  USART_InitStruct.BaudRate            = 115200;//2500000;
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
  DMA_InitStructure.Mode               = LL_DMA_PRIORITY_LOW;
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

  uint32_t bufferpos = sizeof(rxbuf) - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_5);
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