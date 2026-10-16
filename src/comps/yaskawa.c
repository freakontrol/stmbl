#include "yaskawa_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"
#include "yaskawa_crc16.h"

HAL_COMP(yaskawa);

HAL_PIN(pos);
HAL_PIN(error);
HAL_PIN(error_sum);
HAL_PIN(dump);
HAL_PIN(len);
HAL_PIN(off);

HAL_PIN(len2);
HAL_PIN(off2);
HAL_PIN(probe2);

HAL_PIN(len3);
HAL_PIN(len4);
HAL_PIN(len5);

HAL_PIN(probe3);
HAL_PIN(probe4);
HAL_PIN(probe5);

HAL_PIN(send);
HAL_PIN(crc_ok);
HAL_PIN(crc_error);

//TODO: use context
static volatile uint32_t txbuf[128];
static int pos;
static int dfdf;
static volatile char m_data[150];
static volatile char m_data2[150];
static volatile uint16_t tim_data[300];
static LL_DMA_InitTypeDef DMA_InitStructuretx;
static LL_DMA_InitTypeDef DMA_InitStructurerx;
static uint8_t yaskawa_reply[14];
//uint8_t yaskara_reply_len;

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct yaskawa_ctx_t *ctx = (struct yaskawa_ctx_t *)ctx_ptr;
  struct yaskawa_pin_ctx_t *pins = (struct yaskawa_pin_ctx_t *)pin_ptr;

  PIN(len)  = 15;
  PIN(off)  = 64;
  PIN(len3) = 57;
  PIN(len4) = 58;
  PIN(len5) = 59;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct yaskawa_ctx_t *ctx = (struct yaskawa_ctx_t *)ctx_ptr;
  // struct yaskawa_pin_ctx_t *pins = (struct yaskawa_pin_ctx_t *)pin_ptr;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_GPIO_StructInit(&GPIO_InitStruct);

  //TX enable
  GPIO_InitStruct.Pin   = FB0_Z_TXEN_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_Z_TXEN_PORT, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(FB0_Z_TXEN_PORT, FB0_Z_TXEN_PIN);

  //TX
  GPIO_InitStruct.Pin   = FB0_Z_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;  //TODO: default to AF?
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_Z_PORT, &GPIO_InitStruct);
  LL_GPIO_SetAFPin_8_15(FB0_Z_PORT, FB0_Z_PIN, FB0_ENC_TIM_AF);  // TX pin is switched to AF later, PD14

  LL_APB1_GRP1_EnableClock(FB0_ENC_TIM_RCC);


  //manchaster
  //0 -> 01
  //1 -> 10
  //SYNC                framing                         framing
  //0101010101010101010 01111110 111110 111110 111110 1 01111110;
  //010101... are to sync receiver
  //01111110 is hdlc start and end flag for framing
  //0 is appended after 5 ones => transmitted data is 0xffff
  //reply has the same format
  const char request[] = "0101010101010101010 01111110 111110 111110 111110 1 01111110";
  char request_m[sizeof(request) * 2];
  uint32_t tim_a = FB0_Z_PIN << 16;
  uint32_t tim_b = FB0_Z_PIN;

  //encode in manchaster
  int j = 0;
  for(int i = 0; request[i]; i++) {
    if(request[i] == '0') {
      request_m[j++] = '0';
      request_m[j++] = '1';
    } else if(request[i] == '1') {
      request_m[j++] = '1';
      request_m[j++] = '0';
    }
  }
  request_m[j] = '\0';

  //build txbuf for dma
  for(int i = 0; request_m[i]; i++) {
    if(request_m[i] == '0') {
      txbuf[pos++] = tim_a;
    } else if(request_m[i] == '1') {
      txbuf[pos++] = tim_b;
    }
  }
  txbuf[pos++] = tim_a;
  txbuf[pos++] = tim_a;

  DMA_InitStructuretx.Channel            = LL_DMA_CHANNEL_7;
  DMA_InitStructuretx.PeriphOrM2MSrcAddress = (uint32_t)&FB0_Z_PORT->BSRR;  //TODO: change
  DMA_InitStructuretx.MemoryOrM2MDstAddress    = (uint32_t)&txbuf;
  DMA_InitStructuretx.Direction                = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructuretx.NbData         = pos;
  DMA_InitStructuretx.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructuretx.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructuretx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructuretx.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructuretx.Mode               = LL_DMA_MODE_NORMAL;
  DMA_InitStructuretx.Priority           = LL_DMA_PRIORITY_VERYHIGH;
  DMA_InitStructuretx.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructuretx.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructuretx.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructuretx.PeriphBurst    = LL_DMA_PBURST_SINGLE;

  DMA_InitStructurerx.Channel            = LL_DMA_CHANNEL_2;
  DMA_InitStructurerx.PeriphOrM2MSrcAddress = (uint32_t)&FB0_ENC_TIM->CCR3;  //TODO: change
  DMA_InitStructurerx.MemoryOrM2MDstAddress    = (uint32_t)&tim_data;
  DMA_InitStructurerx.Direction                = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructurerx.NbData         = ARRAY_SIZE(tim_data);
  DMA_InitStructurerx.PeriphOrM2MSrcIncMode      = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructurerx.MemoryOrM2MDstIncMode          = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructurerx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStructurerx.MemoryOrM2MDstDataSize     = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStructurerx.Mode               = LL_DMA_MODE_NORMAL;
  DMA_InitStructurerx.Priority           = LL_DMA_PRIORITY_VERYHIGH;
  DMA_InitStructurerx.FIFOMode           = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructurerx.FIFOThreshold      = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructurerx.MemBurst        = LL_DMA_MBURST_SINGLE;
  DMA_InitStructurerx.PeriphBurst    = LL_DMA_PBURST_SINGLE;

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_StructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode       = LL_TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Autoreload            = 20;  // 168 / (20 + 1) = 8MHz
  TIM_TimeBaseStructure.Prescaler         = 0;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  LL_TIM_Init(TIM8, &TIM_TimeBaseStructure);
  LL_TIM_EnableARRPreload(TIM8);
  LL_TIM_EnableDMAReq_UPDATE(TIM8);
  LL_TIM_EnableCounter(TIM8);

  FB0_ENC_TIM->CR1 &= ~TIM_CR1_CEN;
  FB0_ENC_TIM->ARR = 65535;
  FB0_ENC_TIM->CNT = 3300;
  FB0_ENC_TIM->CR1 |= TIM_CR1_CEN;  // enable tim

  dma_stream_stop(DMA1_Stream7);
  LL_DMA_DeInit(DMA1, LL_DMA_STREAM_7);
  LL_DMA_Init(DMA1, LL_DMA_STREAM_7, &DMA_InitStructurerx);

  dma_stream_stop(DMA2_Stream1);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_1);
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &DMA_InitStructuretx);
  dfdf = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct yaskawa_ctx_t *ctx      = (struct yaskawa_ctx_t *)ctx_ptr;
  struct yaskawa_pin_ctx_t *pins = (struct yaskawa_pin_ctx_t *)pin_ptr;

  while(FB0_ENC_TIM->CNT < 3300) {
  }

  int count = ARRAY_SIZE(tim_data) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_7);
  dma_stream_stop(DMA1_Stream7);

  uint16_t bit_time = 15;

  if(count > 80) {
    int pol          = 0;
    int read_counter = 0;
    int counter      = 0;

    for(int i = 0; i < ARRAY_SIZE(yaskawa_reply); i++) {
      yaskawa_reply[i] = 0;
    }

    for(int i = 1; i < count; i++) {
      if(tim_data[i + 1] - tim_data[i] < bit_time) {
        counter++;
      } else if(counter == 10) {
        read_counter = i + 1;
        pol          = 0;
        break;
      } else {
        counter = 0;
      }
    }

    int write_counter2 = 0;
    for(int i = read_counter; i < count; i++) {
      if(tim_data[i + 1] - tim_data[i] < bit_time) {
        i++;
        if(tim_data[i + 1] - tim_data[i] < bit_time) {
          //data[write_counter++] = '0' + pol;
        } else {
          //error
          PIN(error) = 1.0;
          break;
        }

      } else {
        pol = 1 - pol;
        //data[write_counter++] = '0' + pol;
      }

      if(pol == 1) {
        counter++;
        m_data[write_counter2] = '1';
        yaskawa_reply[write_counter2 / 8] |= 1 << (7 - (write_counter2 % 8));
        write_counter2++;
      } else if(counter == 5) {
        counter = 0;
        // unstuff
      } else if(counter == 6) {
        // hldc
        m_data[write_counter2++] = 'H';
        break;
      } else {
        counter                  = 0;
        m_data[write_counter2++] = '0';
      }
    }

    yaskawa_crc16_t crc;
    yaskawa_crc16_t data = (yaskawa_reply[13] & 0xff) | (yaskawa_reply[12] << 8);
    crc                  = yaskawa_crc16_init();
    crc                  = yaskawa_crc16_update(crc, yaskawa_reply, 12);
    crc                  = yaskawa_crc16_finalize(crc);
    if(data == crc) {
      PIN(crc_ok)
      ++;
    } else {
      PIN(crc_error)
      ++;
    }

    m_data[write_counter2] = 0;

    uint32_t pos = 0;
    //extract position data
    for(int i = 0; i < PIN(len); i++) {
      pos += (m_data[i + (int)PIN(off)] == '1') << i;
    }

    uint32_t probe = 0;
    for(int i = 0; i < PIN(len2); i++) {
      probe += (m_data[i + (int)PIN(off2)] == '1') << i;
    }

    PIN(pos)    = (float)pos / (float)(1 << (int)PIN(len)) * M_PI * 2.0 - M_PI;
    PIN(probe2) = probe;

    PIN(probe3) = m_data[(int)PIN(len3)] == '1';
    PIN(probe4) = m_data[(int)PIN(len4)] == '1';
    PIN(probe5) = m_data[(int)PIN(len5)] == '1';
    PIN(error)  = 0.0;

    if(dfdf < 1) {
      for(int i = 0; i < ARRAY_SIZE(m_data2); i++) {
        m_data2[i] = m_data[i];
      }
      dfdf = 1;
    }
  } else {
    PIN(error) = 1.0;
    // error
  }

  // PIN(send) = send;

  FB0_Z_TXEN_PORT->BSRR = FB0_Z_TXEN_PIN;  //TX enable
  FB0_Z_PORT->MODER &= ~GPIO_MODER_MODER14_1;
  FB0_Z_PORT->MODER |= GPIO_MODER_MODER14_0;  //set tx pin to output

  dma_stream_stop(DMA2_Stream1);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_1);
  LL_DMA_Init(DMA2, LL_DMA_STREAM_1, &DMA_InitStructuretx);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);  //transmit request

  TIM8->CR1 &= ~TIM_CR1_CEN;  // disable tim
  TIM8->ARR  = 20;            // 168 / 2 / (9 + 1) = 8.4MHz
  TIM8->DIER = TIM_DIER_UDE;  // cc3 dma
  // TIM8->CCMR2 = 0; // cc3 output
  // TIM8->CCR3 = 1;
  TIM8->CNT = 0;
  TIM8->CR1 |= TIM_CR1_CEN;

  dma_stream_stop(DMA1_Stream7);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);

  FB0_ENC_TIM->CR1 &= ~TIM_CR1_CEN;
  FB0_ENC_TIM->CCMR2 = TIM_CCMR2_CC3S_0;                                // cc3 input ti3
  FB0_ENC_TIM->CCER  = TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP;  // cc3 en, rising edge, falling edge
  FB0_ENC_TIM->ARR   = 65535;
  FB0_ENC_TIM->DIER  = TIM_DIER_CC3DE;  // cc3 dma
  FB0_ENC_TIM->CNT   = 0;
  FB0_ENC_TIM->CCR3  = 0;

  while(!(DMA2->LISR & DMA_LISR_TCIF1))
    ;  //wait for request

  FB0_Z_TXEN_PORT->BSRR = (uint32_t)(FB0_Z_TXEN_PIN) << 16;     //TX disable
  FB0_Z_PORT->MODER &= ~GPIO_MODER_MODER14_0;  //set tx pin to af
  FB0_Z_PORT->MODER |= GPIO_MODER_MODER14_1;

  FB0_ENC_TIM->CR1 |= TIM_CR1_CEN;  // enable tim

  if(PIN(error) > 0.0) {
    PIN(error_sum)
    ++;
  }
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct yaskawa_ctx_t *ctx      = (struct yaskawa_ctx_t *)ctx_ptr;
  struct yaskawa_pin_ctx_t *pins = (struct yaskawa_pin_ctx_t *)pin_ptr;
  if(RISING_EDGE(PIN(dump))) {
    for(int i = 0; i < 14; i++) {
      for(int j = 0; j < 8; j++) {
        printf("%c", m_data2[i * 8 + j]);
      }
      printf("|");
    }
    printf("\n");
    dfdf = 0;
  }
}

hal_comp_t yaskawa_comp_struct = {
    .name      = "yaskawa",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct yaskawa_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
