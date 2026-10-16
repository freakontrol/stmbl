#include "encs_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"

/*
Sanyo denki absolute encoder, tested with PA035C

10 0010001000000101 10 0001110110110000 10 0011011101001010 11

10001000100000010110000111011011000010001101110100101011
10....word A......10.....word B.....10......word C....11

2.5mbit
3 16 bit words

word A:
constant, alarm/model?

word B:
16 bit pos, LSB first

word C:
1. bit: MSB pos bit
2..15: checksum/multiturn?
*/

HAL_COMP(encs);

HAL_PIN(pos);
HAL_PIN(error);
HAL_PIN(state);

static volatile uint32_t request_buf[24];
static volatile uint16_t tim_data[300];

#pragma pack(push, 1)
typedef union {
  uint8_t enc_data[10];
  struct {
    uint32_t start1 : 1;
    uint32_t sync : 3;
    uint32_t eax : 3;
    uint32_t cc : 5;
    uint32_t fixed : 1;
    uint32_t es : 4;
    uint32_t stop1 : 1;
    uint32_t start2 : 1;
    uint32_t pos0_15 : 16;
    uint32_t stop2 : 1;
    uint32_t start3 : 1;
    uint32_t pos16 : 1;
    uint32_t mpos : 7;
    uint32_t crc : 8;
    uint32_t stop3 : 1;
  } sanyo;
  struct {
    uint32_t start1 : 1;
    uint32_t d0 : 8;
    uint32_t d1 : 8;
    uint32_t stop1 : 1;
    uint32_t start2 : 1;
    uint32_t d2 : 8;
    uint32_t d3 : 8;
    uint32_t stop2 : 1;
    uint32_t start3 : 1;
    uint32_t d4 : 8;
    uint32_t crc : 8;
    uint32_t stop3 : 1;
  } data;
} sanyo_t;
#pragma pack(pop)

sanyo_t data;

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct encs_ctx_t *ctx = (struct encs_ctx_t *)ctx_ptr;
  // struct encs_pin_ctx_t *pins = (struct encs_pin_ctx_t *)pin_ptr;

  //TX enable
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.Pin   = FB0_Z_TXEN_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_Z_TXEN_PORT, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(FB0_Z_TXEN_PORT, FB0_Z_TXEN_PIN);

  //TX
  GPIO_InitStruct.Pin   = FB0_Z_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_Z_PORT, &GPIO_InitStruct);
  LL_GPIO_SetAFPin_8_15(FB0_Z_PORT, FB0_Z_PIN, FB0_ENC_TIM_AF);  // TX pin is switched to AF later, PD14

  //RX DMA
  LL_DMA_InitTypeDef dma_rx_config;
  LL_DMA_StructInit(&dma_rx_config);
  dma_rx_config.Channel            = LL_DMA_CHANNEL_2;
  dma_rx_config.PeriphOrM2MSrcAddress = (uint32_t)&FB0_ENC_TIM->CCR3;
  dma_rx_config.MemoryOrM2MDstAddress    = (uint32_t)&tim_data;
  dma_rx_config.Direction                = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_rx_config.NbData         = ARRAY_SIZE(tim_data);
  dma_rx_config.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  dma_rx_config.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  dma_rx_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  dma_rx_config.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_HALFWORD;
  dma_rx_config.Mode               = LL_DMA_MODE_NORMAL;
  dma_rx_config.Priority           = LL_DMA_PRIORITY_VERYHIGH;
  dma_rx_config.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  dma_rx_config.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  dma_rx_config.MemBurst        = LL_DMA_MBURST_SINGLE;
  dma_rx_config.PeriphBurst    = LL_DMA_PBURST_SINGLE;

  dma_stream_stop(DMA1_Stream7);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_7);
  LL_DMA_Init(DMA1, LL_DMA_STREAM_7, &dma_rx_config);

  //timer setup
  LL_APB1_GRP1_EnableClock(FB0_ENC_TIM_RCC);
  FB0_ENC_TIM->CR1 &= ~TIM_CR1_CEN;
  FB0_ENC_TIM->CCMR2 = TIM_CCMR2_CC3S_0;                                // cc3 input ti3
  FB0_ENC_TIM->CCER  = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP;  // cc3 en, rising edge, falling edge
  FB0_ENC_TIM->ARR   = 65535;
  FB0_ENC_TIM->DIER  = TIM_DIER_CC3DE;  // cc3 dma
  FB0_ENC_TIM->CNT   = 0;
  FB0_ENC_TIM->CCR3  = 0;

  const uint32_t tx_high = FB0_Z_PIN;
  const uint32_t tx_low  = FB0_Z_PIN << 16;
  int pos                = 0;
  request_buf[pos++]     = tx_high;
  request_buf[pos++]     = tx_high;
  request_buf[pos++]     = tx_high;
  request_buf[pos++]     = tx_high;

  request_buf[pos++] = tx_low;  //start bit

  request_buf[pos++] = tx_low;   //sync
  request_buf[pos++] = tx_high;  //sync
  request_buf[pos++] = tx_low;   //sync

  request_buf[pos++] = tx_low;  //frame
  request_buf[pos++] = tx_low;  //frame
  //encoder address
  request_buf[pos++] = tx_low;  //EA
  request_buf[pos++] = tx_low;  //EA
  request_buf[pos++] = tx_low;  //EA
  //command code = CDF1 = Absolute lower 24-bit data request
  request_buf[pos++] = tx_high;  //CC
  request_buf[pos++] = tx_low;   //CC
  request_buf[pos++] = tx_low;   //CC
  request_buf[pos++] = tx_low;   //CC
  request_buf[pos++] = tx_low;   //CC
  //crc of 10 bit, frame code to command code
  request_buf[pos++] = tx_low;   //crc
  request_buf[pos++] = tx_low;   //crc
  request_buf[pos++] = tx_high;  //crc

  request_buf[pos++] = tx_high;  //stop bit
  request_buf[pos++] = tx_high;
  request_buf[pos++] = tx_high;

  //DMA tx config
  LL_DMA_InitTypeDef dma_tx_config;
  LL_DMA_StructInit(&dma_tx_config);
  dma_tx_config.Channel            = LL_DMA_CHANNEL_7;  //TIM8_UP
  dma_tx_config.PeriphOrM2MSrcAddress = (uint32_t)&FB0_Z_PORT->BSRR;
  dma_tx_config.MemoryOrM2MDstAddress    = (uint32_t)&request_buf;
  dma_tx_config.Direction                = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_tx_config.NbData         = pos;
  dma_tx_config.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  dma_tx_config.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  dma_tx_config.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  dma_tx_config.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_WORD;
  dma_tx_config.Mode               = LL_DMA_MODE_NORMAL;
  dma_tx_config.Priority           = LL_DMA_PRIORITY_VERYHIGH;
  dma_tx_config.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  dma_tx_config.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  dma_tx_config.MemBurst        = LL_DMA_MBURST_SINGLE;
  dma_tx_config.PeriphBurst    = LL_DMA_PBURST_SINGLE;
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_1);
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &dma_tx_config);

  //TIM8 tx bitbang timer
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_StructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode       = LL_TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Autoreload            = 32;  //14MHz
  TIM_TimeBaseStructure.Prescaler         = 1;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  LL_TIM_Init(TIM8, &TIM_TimeBaseStructure);
  LL_TIM_EnableARRPreload(TIM8);
  LL_TIM_EnableDMAReq_UPDATE(TIM8);
  LL_TIM_EnableCounter(TIM8);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct encs_ctx_t *ctx      = (struct encs_ctx_t *)ctx_ptr;
  struct encs_pin_ctx_t *pins = (struct encs_pin_ctx_t *)pin_ptr;

  int count = ARRAY_SIZE(tim_data) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_7);
  dma_stream_stop(DMA1_Stream7);

  for(int i = 0; i < 10; i++) {
    data.enc_data[i] = 0;
  }

  uint8_t bits_sum = 0;
  for(int i = 2; i < count; i++) {  //each capture form dma
    //calculate time between edges
    uint16_t diff = tim_data[i] - tim_data[i - 1];
    //number of bits to set
    //1 bit = 82e6/2.5e6
    int bits = (float)diff / (float)33.0f + 0.5f;
    if(i % 2 != 0) {  //line starts high, set every odd numbered captures to 1
      for(int j = bits_sum; j < bits + bits_sum; j++) {
        data.enc_data[j / 8] |= (1 << j % 8);
      }
    }
    bits_sum += bits;
  }
  //set remaining bits to 1
  for(int j = bits_sum; j < 54; j++) {
    data.enc_data[j / 8] |= (1 << j % 8);
  }

  if(!data.sanyo.start1 && !data.sanyo.start2 && !data.sanyo.start3 && data.sanyo.stop1 && data.sanyo.stop2 && data.sanyo.stop3) {
    PIN(pos)   = ((float)data.sanyo.pos0_15 / 65536.0f + data.sanyo.pos16) * M_PI - M_PI;
    PIN(error) = 0;
    PIN(state) = 3;
  } else {
    PIN(error) = 1;
    PIN(state) = 0;
  }

  //request
  FB0_Z_TXEN_PORT->BSRR = FB0_Z_TXEN_PIN;  //TX enable
  FB0_Z_PORT->MODER &= ~GPIO_MODER_MODER14_1;
  FB0_Z_PORT->MODER |= GPIO_MODER_MODER14_0;  //set tx pin to output
  TIM8->ARR = 32;                             //2.545 Mhz
  dma_stream_stop(DMA2_Stream1);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
  //wait for DMA transfer complete
  while(LL_DMA_IsActiveFlag_TC1(DMA2) == RESET)
    ;
  FB0_Z_TXEN_PORT->BSRR = (uint32_t)(FB0_Z_TXEN_PIN) << 16;     //TX disable
  FB0_Z_PORT->MODER &= ~GPIO_MODER_MODER14_0;  //set tx pin to af
  FB0_Z_PORT->MODER |= GPIO_MODER_MODER14_1;

  //reset timer
  FB0_ENC_TIM->CR1 &= ~TIM_CR1_CEN;
  FB0_ENC_TIM->CNT      = 0;
  //volatile uint32_t foo = FB0_ENC_TIM->CCR3;
  FB0_ENC_TIM->CCR3     = 0;
  FB0_ENC_TIM->CR1 |= TIM_CR1_CEN;  // enable tim

  //start rx DMA
  dma_stream_stop(DMA1_Stream7);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);
}

hal_comp_t encs_comp_struct = {
    .name      = "encs",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct encs_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
