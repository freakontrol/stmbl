#include "usart_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"

HAL_COMP(usart);

HAL_PIN(req);
HAL_PIN(print);
HAL_PIN(freq);
HAL_PIN(dma);  //dma transfers left

HAL_PIN(pos_offset);
HAL_PIN(pos_len);
HAL_PIN(pos);

struct usart_ctx_t {
  uint8_t rxbuf[16];
  uint8_t rxbuf2[16];
  int8_t print;
  uint8_t dma;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;
  PIN(freq)                    = 2500000;
  PIN(req)                     = 0x2A;  // 0x32
  PIN(pos_offset)              = 2;
  PIN(pos_len)                 = 20;
}


static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx = (struct usart_ctx_t *)ctx_ptr;
  // struct usart_pin_ctx_t * pins = (struct usart_pin_ctx_t *)pin_ptr;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
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
  DMA_InitStructure.Mode               = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority           = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst    = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &DMA_InitStructure);

  LL_USART_EnableDMAReq_RX(USART6);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx      = (struct usart_ctx_t *)ctx_ptr;
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;

  LL_USART_InitTypeDef USART_InitStruct;

  LL_USART_StructInit(&USART_InitStruct);

  PIN(dma) = LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);

  if(PIN(print) <= 1 && ctx->print == -1) {
    ctx->print = 0;
  }

  union {
    uint64_t pos;
    uint8_t data[8];
  } foo;

  for(int i = 0; i < PIN(pos_len); i++) {
    foo.data[i] = ctx->rxbuf[i + (int)PIN(pos_offset)];
  }

  foo.pos &= (2 << (int)PIN(pos_len)) - 1;

  PIN(pos) = foo.pos * 2.0 * M_PI / pow(2, PIN(pos_len)) - M_PI;

  if(PIN(print) > 0 && ctx->print == 0) {
    for(int i = 0; i < ARRAY_SIZE(ctx->rxbuf); i++) {
      ctx->rxbuf2[i] = ctx->rxbuf[i];
      ctx->rxbuf[i]  = 0;
    }
    ctx->dma   = PIN(dma);
    ctx->print = 1;
    PIN(print) = 0;
  }

  LL_USART_Disable(USART6);

  USART_InitStruct.BaudRate            = PIN(freq);
  USART_InitStruct.DataWidth          = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection                = LL_USART_DIRECTION_TX_RX;
  LL_USART_Init(USART6, &USART_InitStruct);

  /* Enable the USART */
  LL_USART_Enable(USART6);
  SET_BIT(USART6->CR3, USART_CR3_HDSEL);  // half duplex


  //TODO: irq here will cause problems
  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx enable
  (USART6)->DR = (uint8_t)PIN(req);
  while(LL_USART_IsActiveFlag_TC(USART6) == RESET)
    ;
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_15);  //tx disable

  //start rx dma
  dma_stream_stop(DMA2_Stream1);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}


static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct usart_ctx_t *ctx      = (struct usart_ctx_t *)ctx_ptr;
  struct usart_pin_ctx_t *pins = (struct usart_pin_ctx_t *)pin_ptr;

  if(ctx->print > 0) {
    ctx->print = -1;
    printf("req: 0x%02X: rx: %u : ", (uint8_t)PIN(req), ctx->dma);
    for(int i = 0; i < ARRAY_SIZE(ctx->rxbuf); i++) {
      for(int j = 0; j < 8; j++) {
        if(ctx->rxbuf2[i] & (1 << j % 8)) {
          printf("1");
        } else {
          printf("0");
        }
      }
      printf(" ");
      //printf("%02X ", ctx->rxbuf2[i]);
    }
    printf("\n");
  }
}

hal_comp_t usart_comp_struct = {
    .name      = "usart",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct usart_ctx_t),
    .pin_count = sizeof(struct usart_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
