//
//  setup.c
//  test
//
//  Created by Rene on 09/12/13.
//  Copyright (c) 2013 Rene Hopf. All rights reserved.
//

#include "setup.h"
#include "usbd_cdc_if.h"
#include "defines.h"
#include "f4_clock.h"

LL_RCC_ClocksTypeDef RCC_Clocks;
volatile uint32_t ADC_DMA_Buffer0[ADC_SAMPLES_IN_RT];  //240
volatile uint32_t ADC_DMA_Buffer1[ADC_SAMPLES_IN_RT];

void setup() {
  clock_init();  // 168 MHz, from the HSI SystemInit() left us on

  //Enable clocks
  //TODO: small f4 does not have GPIOE
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD | LL_AHB1_GRP1_PERIPH_GPIOE | LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMA2 | LL_AHB1_GRP1_PERIPH_CRC);

  NVIC_SetPriorityGrouping(3);  // 4 bits preemption, 0 bits subpriority

  // systick timer, before usb_init(): the HAL USB driver waits with HAL_Delay()
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  //systick prio

  NVIC_SetPriority(SysTick_IRQn, 14);

  setup_res();
  usb_init();

  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit(&GPIO_InitStructure);
  // messpin
  GPIO_InitStructure.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull       = LL_GPIO_PULL_NO;

  //fan
  GPIO_InitStructure.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_0;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

}

static const uint32_t adc_reg_rank[16] = {
    LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4,
    LL_ADC_REG_RANK_5, LL_ADC_REG_RANK_6, LL_ADC_REG_RANK_7, LL_ADC_REG_RANK_8,
    LL_ADC_REG_RANK_9, LL_ADC_REG_RANK_10, LL_ADC_REG_RANK_11, LL_ADC_REG_RANK_12,
    LL_ADC_REG_RANK_13, LL_ADC_REG_RANK_14, LL_ADC_REG_RANK_15, LL_ADC_REG_RANK_16};

// rank is 1 based, like the old ADC_RegularChannelConfig()
static void adc_regular_channel_config(ADC_TypeDef *adc, uint32_t chan, int rank, uint32_t sample_time) {
  LL_ADC_REG_SetSequencerRanks(adc, adc_reg_rank[rank - 1], chan);
  LL_ADC_SetChannelSamplingTime(adc, chan, sample_time);
}

// Setup Resolver Interface
// master timer triggers ADC1,ADC2 via OC, and slave timer via TRGO at 1.2MHz
// slave timer OC generates resolver reference signal at 10kHz, phase can be adjusted by oc value
// DMA2 moves ADC_ANZ samples to memory, generates transfer complete interrupt at 5kHz
void setup_res() {
  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;
  //master timer
  LL_APB1_GRP1_EnableClock(TIM_MASTER_RCC);
  LL_TIM_StructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode       = LL_TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Autoreload        = ADC_TIMER_FREQ / ADC_TRIGGER_FREQ - 1;  //70 1.2MHz
  TIM_TimeBaseStructure.Prescaler         = 0;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  LL_TIM_Init(TIM_MASTER, &TIM_TimeBaseStructure);
  LL_TIM_EnableARRPreload(TIM_MASTER);
  LL_TIM_SetTriggerOutput(TIM_MASTER, LL_TIM_TRGO_UPDATE);  // trigger ADC

  //oc for adc trigger
  LL_TIM_OC_StructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.OCMode       = LL_TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OCState      = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.OCNState     = LL_TIM_OCSTATE_DISABLE;
  TIM_OCInitStructure.CompareValue = 1;
  TIM_OCInitStructure.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState  = LL_TIM_OCIDLESTATE_HIGH;
  //ADC trigger OC depends on timer
  LL_TIM_OC_Init(TIM_MASTER, TIM_MASTER_ADC_OC_CH, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(TIM_MASTER, TIM_MASTER_ADC_OC_CH);
  // TIM_MASTER is a general purpose timer, no MOE bit to set

  //slave timer triggers frt
  LL_APB1_GRP1_EnableClock(TIM_SLAVE_RCC);
  LL_TIM_StructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode       = LL_TIM_COUNTERMODE_UP;
  TIM_TimeBaseStructure.Autoreload        = ADC_TRIGGER_FREQ / FRT_FREQ - 1;  //60 20kHz
  TIM_TimeBaseStructure.Prescaler         = 0;
  TIM_TimeBaseStructure.RepetitionCounter = 0;
  LL_TIM_Init(TIM_SLAVE, &TIM_TimeBaseStructure);
  //Rising edges of the selected trigger (TRGI) clock the counter, clk = TIM_MASTER trigger out
  LL_TIM_SetTriggerInput(TIM_SLAVE, TIM_SLAVE_ITR);
  LL_TIM_SetClockSource(TIM_SLAVE, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_EnableARRPreload(TIM_SLAVE);
  TIM_SLAVE->CNT = (TIM_SLAVE->ARR + 1) / 2;
  LL_TIM_EnableCounter(TIM_SLAVE);

  /* ADC clock enable */
  LL_APB2_GRP1_EnableClock(FB0_SIN_ADC_RCC | FB0_COS_ADC_RCC);

#ifdef FB1
  LL_APB2_GRP1_EnableClock(FB1_SIN_ADC_RCC | FB1_COS_ADC_RCC);
#endif

  //Analog pin configuration
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.Pin  = FB0_SIN_PIN;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FB0_SIN_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB0_COS_PIN;
  LL_GPIO_Init(FB0_COS_PORT, &GPIO_InitStructure);

#ifdef FB1
  GPIO_InitStructure.Pin = FB1_SIN_PIN;
  LL_GPIO_Init(FB1_SIN_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = FB1_COS_PIN;
  LL_GPIO_Init(FB1_COS_PORT, &GPIO_InitStructure);
#endif

  //ADC structure configuration
  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_ADC);
  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_ADC);
  ADC_TypeDef *const adcs[2] = {FB0_SIN_ADC, FB0_COS_ADC};
  for(int i = 0; i < 2; i++) {
    LL_ADC_SetResolution(adcs[i], LL_ADC_RESOLUTION_12B);         //Input voltage is converted into a 12bit number giving a maximum value of 4096
    LL_ADC_SetDataAlignment(adcs[i], LL_ADC_DATA_ALIGN_RIGHT);    //data converted will be shifted to right
    LL_ADC_SetSequencersScanMode(adcs[i], LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_REG_SetContinuousMode(adcs[i], LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetSequencerLength(adcs[i], (ADC_OVER_FB0 + ADC_OVER_FB1 - 1) << ADC_SQR1_L_Pos);
  }
  LL_ADC_REG_SetTriggerSource(FB0_SIN_ADC, TIM_MASTER_ADC);  //trigger on rising edge of TIM_MASTER oc
  // F4 LL SetTriggerSource only writes EXTSEL, the edge (EXTEN) is set here
  LL_ADC_REG_StartConversionExtTrig(FB0_SIN_ADC, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetTriggerSource(FB0_COS_ADC, LL_ADC_REG_TRIG_SOFTWARE);  // slave in dual mode

  ADC_Common_TypeDef *adc_common = __LL_ADC_COMMON_INSTANCE(FB0_SIN_ADC);
  LL_ADC_SetMultimode(adc_common, LL_ADC_MULTI_DUAL_REG_SIMULT);
  LL_ADC_SetCommonClock(adc_common, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
  LL_ADC_SetMultiTwoSamplingDelay(adc_common, LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES);

  for(int i = 1; i <= ADC_OVER_FB0; i++) {
    adc_regular_channel_config(FB0_SIN_ADC, FB0_SIN_ADC_CHAN, i, RES_SampleTime);
    adc_regular_channel_config(FB0_COS_ADC, FB0_COS_ADC_CHAN, i, RES_SampleTime);
  }

#ifdef FB1
  for(int i = ADC_OVER_FB0 + 1; i <= ADC_OVER_FB0 + ADC_OVER_FB1; i++) {
    adc_regular_channel_config(FB1_SIN_ADC, FB1_SIN_ADC_CHAN, i, RES_SampleTime);
    adc_regular_channel_config(FB1_COS_ADC, FB1_COS_ADC_CHAN, i, RES_SampleTime);
  }
#endif

  // discontinuous mode, one conversion per trigger
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_1RANK);
  LL_ADC_REG_SetSequencerDiscont(ADC2, LL_ADC_REG_SEQ_DISCONT_1RANK);

  // DMA mode 2, DMA requests continue after the last transfer
  LL_ADC_SetMultiDMATransfer(adc_common, LL_ADC_MULTI_REG_DMA_UNLMT_2);

  // DMA-Disable
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  while(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_0)) {
  }
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_0);

  // DMA2-Config
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.Channel                = LL_DMA_CHANNEL_0;
  DMA_InitStructure.PeriphOrM2MSrcAddress  = (uint32_t)&ADC->CDR;
  DMA_InitStructure.MemoryOrM2MDstAddress  = (uint32_t)ADC_DMA_Buffer0;
  DMA_InitStructure.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.NbData                 = ARRAY_SIZE(ADC_DMA_Buffer0);
  DMA_InitStructure.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode                   = LL_DMA_MODE_CIRCULAR;
  DMA_InitStructure.Priority               = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst               = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst            = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMA_InitStructure);

  LL_DMA_SetMemory1Address(DMA2, LL_DMA_STREAM_0, (uint32_t)ADC_DMA_Buffer1);
  LL_DMA_SetCurrentTargetMem(DMA2, LL_DMA_STREAM_0, LL_DMA_CURRENTTARGETMEM0);
  LL_DMA_EnableDoubleBufferMode(DMA2, LL_DMA_STREAM_0);

  //HAL Fast realtime irq 20kHz
  NVIC_SetPriority(TIM_SLAVE_IRQ, 0);
  NVIC_EnableIRQ(TIM_SLAVE_IRQ);

  //ADC overrun would stop the rt silently, see ADC_IRQHandler
  LL_ADC_EnableIT_OVR(FB0_SIN_ADC);
  LL_ADC_EnableIT_OVR(FB0_COS_ADC);
  NVIC_SetPriority(ADC_IRQn, 2);
  NVIC_EnableIRQ(ADC_IRQn);

  //HAL Realtime irq 5kHz
  NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

  // ADCs on only once the stream takes their requests, so no conversion can overrun first
  LL_ADC_Enable(FB0_SIN_ADC);
  LL_ADC_Enable(FB0_COS_ADC);
}
