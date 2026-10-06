#include "io4_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#include "hw/hw.h"
#include "main.h"
#include "common.h"

// TIM9 is on APB2 (84 MHz, prescaler 2), so it counts at 2 x 84 MHz
// (RM0090 6.2, Figure 21 note 2)
#define TIM9_CLK 168000000

HAL_COMP(io);

HAL_PIN(fan);

HAL_PIN(state);
HAL_PIN(fault);

//outputs
HAL_PIN(out0);
HAL_PIN(out1);
HAL_PIN(out2);

HAL_PIN(out_freq);
HAL_PIN(out_arr);
HAL_PIN(out_cnt);
HAL_PIN(out_ccr1);

HAL_PIN(in0);    //input 0, analog
HAL_PIN(in1);    //input 1, analog
HAL_PIN(ind0);   //input 0, digital
HAL_PIN(ind1);   //input 1, digital
HAL_PIN(ind0n);  //input 0 inverted
HAL_PIN(ind1n);  //input 0 inverted
HAL_PIN(th0);    //voltage threshold in0
HAL_PIN(th1);    //voltage threshold in1

HAL_PIN(CTX);
HAL_PIN(CRX);
HAL_PIN(C12);
HAL_PIN(C36);
HAL_PIN(C54);
HAL_PIN(C78);
HAL_PIN(cmd_remap);

HAL_PIN(swd_remap);
HAL_PIN(DIO);
HAL_PIN(CK);

//rj45 leds
HAL_PIN(fb0g);
HAL_PIN(fb0y);

HAL_PIN(fb1g);
HAL_PIN(fb1y);

HAL_PIN(cmdg);
HAL_PIN(cmdy);

HAL_PIN(fb0);
HAL_PIN(fb1);
HAL_PIN(fbd0);
HAL_PIN(fbd1);
HAL_PIN(fbd0n);
HAL_PIN(fbd1n);
HAL_PIN(fbth0);
HAL_PIN(fbth1);

//a,b,z inputs of fb0 and fb1
HAL_PIN(fb0a);
HAL_PIN(fb0b);
HAL_PIN(fb0z);

HAL_PIN(fb1a);
HAL_PIN(fb1b);
HAL_PIN(fb1z);

HAL_PIN(fbsd);  //fb shutdown

HAL_PIN(master_arr);
HAL_PIN(clock_scale);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;
  PIN(th0)                  = 12.0;
  PIN(th1)                  = 12.0;
  PIN(fbth0)                = 2.5;
  PIN(fbth1)                = 2.5;
  PIN(out_freq)             = 15000;
  PIN(master_arr)           = TIM3->ARR;
  PIN(clock_scale)          = 1.0;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct io_ctx_t * ctx = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  LL_GPIO_InitTypeDef GPIO_InitStructure;
  LL_GPIO_StructInit(&GPIO_InitStructure);

  //**** ADC3 for analog input and fb temperature
  //TODO: ADC calibration?
  GPIO_InitStructure.Pin   = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.Pin   = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  // out1, out2 pwm tim9
  GPIO_InitStructure.Pin   = LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
  GPIO_InitStructure.Mode  = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  if(PIN(swd_remap) > 0) {
    GPIO_InitStructure.Pin   = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    GPIO_InitStructure.Mode  = LL_GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
    LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  }

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC3);
  LL_ADC_SetResolution(ADC3, LL_ADC_RESOLUTION_12B);       //Input voltage is converted into a 12bit number giving a maximum value of 4096
  LL_ADC_SetDataAlignment(ADC3, LL_ADC_DATA_ALIGN_RIGHT);  //data converted will be shifted to right
  LL_ADC_SetSequencersScanMode(ADC3, LL_ADC_SEQ_SCAN_ENABLE);
  LL_ADC_REG_SetContinuousMode(ADC3, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetSequencerLength(ADC3, LL_ADC_REG_SEQ_SCAN_DISABLE);  // 1 regular conversion, not used
  // injected group, software triggered
  LL_ADC_INJ_SetSequencerLength(ADC3, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_10);
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_11);
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_12);
  LL_ADC_INJ_SetSequencerRanks(ADC3, LL_ADC_INJ_RANK_4, LL_ADC_CHANNEL_13);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_Enable(ADC3);
  LL_ADC_INJ_StartConversionSWStart(ADC3);
  //**** ADC3 end

  GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;

  //fan
  GPIO_InitStructure.Pin = LL_GPIO_PIN_3;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  //red
  GPIO_InitStructure.Pin = LL_GPIO_PIN_3;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //yellow
  GPIO_InitStructure.Pin = LL_GPIO_PIN_4;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //green
  GPIO_InitStructure.Pin = LL_GPIO_PIN_5;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //in1 led
  GPIO_InitStructure.Pin = LL_GPIO_PIN_0;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  //in0 led
  GPIO_InitStructure.Pin = LL_GPIO_PIN_1;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  //out0
  GPIO_InitStructure.Pin = LL_GPIO_PIN_4;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  // //out1
  // GPIO_InitStructure.Pin = LL_GPIO_PIN_5;
  // LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  // //out2
  // GPIO_InitStructure.Pin = LL_GPIO_PIN_6;
  // LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

  TIM9->ARR   = TIM9_CLK / 300 / 15000 - 1;  // 15000Hz
  TIM9->PSC   = 300 - 1;  // 168e6 / 300 = 560000Hz max freq
  TIM9->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // pwm mode 1
  TIM9->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;                                              // cc1, cc2 enable
  TIM9->CCR1  = 0;
  TIM9->CCR2  = 0;
  TIM9->CR1   = TIM_CR1_ARPE;  // preloade
  TIM9->CR1 |= TIM_CR1_CEN;    // counter enable

  //fb0 green
  GPIO_InitStructure.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //fb0 yellow
  GPIO_InitStructure.Pin = LL_GPIO_PIN_9;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //fb1 green
  GPIO_InitStructure.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  //fb1 yellow
  GPIO_InitStructure.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOE, &GPIO_InitStructure);

  //cmd
  // GPIO_InitStructure.Pin   = CMD_C_EN_PIN;
  // LL_GPIO_Init(CMD_C_EN_PORT, &GPIO_InitStructure);
  //
  // GPIO_InitStructure.Pin   = CMD_D_EN_PIN;
  // LL_GPIO_Init(CMD_D_EN_PORT, &GPIO_InitStructure);

  //cmd yellow
  GPIO_InitStructure.Pin = LL_GPIO_PIN_6;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //cmd green
  GPIO_InitStructure.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  //fb 5v enable
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
  GPIO_InitStructure.Pin = LL_GPIO_PIN_13;
  LL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  if(PIN(cmd_remap) > 0) {
    // CMD EN
    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
    GPIO_InitStructure.Pin   = CMD_A_EN_PIN;
    LL_GPIO_Init(CMD_A_EN_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CMD_B_EN_PIN;
    LL_GPIO_Init(CMD_B_EN_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CMD_C_EN_PIN;
    LL_GPIO_Init(CMD_C_EN_PORT, &GPIO_InitStructure);

    LL_GPIO_SetOutputPin(CMD_A_EN_PORT, CMD_A_EN_PIN);
    LL_GPIO_SetOutputPin(CMD_B_EN_PORT, CMD_B_EN_PIN);
    LL_GPIO_SetOutputPin(CMD_C_EN_PORT, CMD_C_EN_PIN);
  }
}

#include "../shared/hw_math.h"

#define ARES 4096.0
#define AREF 3.3

static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;
  if(PIN(clock_scale) > 1.01) {
    TIM3->ARR = PIN(master_arr) + 1;
  } else if(PIN(clock_scale) < 0.99) {
    TIM3->ARR = PIN(master_arr) - 1;
  } else {
    TIM3->ARR = PIN(master_arr);
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct io_ctx_t * ctx = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;
  uint32_t red              = 0;
  uint32_t green            = 0;
  uint32_t yellow           = 0;

  //TODO: unit conversion
  //TODO: check if adc sample complete?
  float in0 = V2(ADC2V(LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_1)), 3.3, 1000.0, 10000.0, 1000.0);
  float in1 = V2(ADC2V(LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_2)), 3.3, 1000.0, 10000.0, 1000.0);
  PIN(in0)  = in0;
  PIN(in1)  = in1;

  if(in0 > PIN(th0) + 0.1) {
    PIN(ind0)  = 1.0;
    PIN(ind0n) = 0.0;
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_1);
  } else if(in0 < PIN(th0) - 0.1) {
    PIN(ind0)  = 0.0;
    PIN(ind0n) = 1.0;
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_1);
  }

  if(in1 > PIN(th1) + 0.1) {
    PIN(ind1)  = 1.0;
    PIN(ind1n) = 0.0;
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_0);
  } else if(in1 < PIN(th1) - 0.1) {
    PIN(ind1)  = 0.0;
    PIN(ind1n) = 1.0;
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_0);
  }


  in0      = V3(ADC2V(LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_3)), 10000.0, 1000.0);
  in1      = V3(ADC2V(LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_4)), 10000.0, 1000.0);
  PIN(fb0) = in0;
  PIN(fb1) = in1;

  LL_ADC_INJ_StartConversionSWStart(ADC3);

  if(in0 > PIN(fbth0) + 0.1) {
    PIN(fbd0)  = 1.0;
    PIN(fbd0n) = 0.0;
  } else if(in0 < PIN(fbth0) - 0.1) {
    PIN(fbd0)  = 0.0;
    PIN(fbd0n) = 1.0;
  }

  if(in1 > PIN(fbth1) + 0.1) {
    PIN(fbd1)  = 1.0;
    PIN(fbd1n) = 0.0;
  } else if(in1 < PIN(fbth1) - 0.1) {
    PIN(fbd1)  = 0.0;
    PIN(fbd1n) = 1.0;
  }

  if(PIN(swd_remap) > 0) {
    PIN(DIO) = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_13);
    PIN(CK)  = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_14);
  }

  switch((state_t)PIN(state)) {
    case DISABLED:
      red    = 0;
      green  = 0;
      yellow = 1;
      break;

    case ENABLED:
      red    = 0;
      green  = 1;
      yellow = 0;
      break;

    case PHASING:
      red    = 0;
      green  = 1;
      yellow = 1;
      break;

    case SOFT_FAULT:
      red    = BLINK((int)PIN(fault));
      green  = 0;
      yellow = 0;
      break;

    case HARD_FAULT:
      red    = BLINK((int)PIN(fault));
      green  = BLINK((int)PIN(fault));
      yellow = BLINK((int)PIN(fault));
      break;

    case LED_TEST:
      red    = 1;
      green  = 1;
      yellow = 1;
      break;
  }

  if(red > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_3);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_3);

  if(yellow > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_4);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_4);

  if(green > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_5);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_5);

  if(PIN(out0) > 0)
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_4);
  else
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_4);

  // if(PIN(out1) > 0)
  //   LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_5);
  // else
  //   LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_5);

  // if(PIN(out2) > 0)
  //   LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_6);
  // else
  //   LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_6);

  // the period is ARR + 1 counts; CCR = ARR + 1 is 100 % duty
  uint32_t pwm_ticks = MAX(TIM9_CLK / 300 / MAX(PIN(out_freq), 10), 2);
  TIM9->ARR  = pwm_ticks - 1;
  TIM9->CCR1 = CLAMP(PIN(out1), 0, 1) * pwm_ticks;
  TIM9->CCR2 = CLAMP(PIN(out2), 0, 1) * pwm_ticks;

  PIN(out_cnt)  = TIM9->CNT;
  PIN(out_arr)  = TIM9->ARR;
  PIN(out_ccr1) = TIM9->CCR1;

  if(PIN(fb0g) > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_8);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8);

  if(PIN(fb0y) > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_9);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_9);

  if(PIN(fb1g) > 0)
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_7);
  else
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_7);

  if(PIN(fb1y) > 0)
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_8);
  else
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_8);

  // if(PIN(cmdc_en) > 0){
  //    LL_GPIO_SetOutputPin(CMD_C_EN_PORT, CMD_C_EN_PIN);
  //    GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  //    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  //    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  //    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  //    GPIO_InitStructure.Pin = CMD_C_PIN;
  //    LL_GPIO_Init(CMD_C_PORT, &GPIO_InitStructure);
  //    if(PIN(cmdc) > 0){
  //       LL_GPIO_SetOutputPin(CMD_C_PORT, CMD_C_PIN);
  //    }
  //    else{
  //       LL_GPIO_ResetOutputPin(CMD_C_PORT, CMD_C_PIN);
  //    }
  // }
  // else{
  //    GPIO_InitStructure.Mode  = LL_GPIO_MODE_INPUT;
  //    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  //    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  //    GPIO_InitStructure.Pin = CMD_C_PIN;
  //    LL_GPIO_Init(CMD_C_PORT, &GPIO_InitStructure);
  //    LL_GPIO_ResetOutputPin(CMD_C_EN_PORT, CMD_C_EN_PIN);
  //    PIN(cmdc) = (CMD_C_PORT->IDR & CMD_C_PIN) > 0;
  // }
  //
  // if(PIN(cmdd_en) > 0){
  //    LL_GPIO_SetOutputPin(CMD_D_EN_PORT, CMD_D_EN_PIN);
  //    GPIO_InitStructure.Mode  = LL_GPIO_MODE_OUTPUT;
  //    GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  //    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  //    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  //    GPIO_InitStructure.Pin = CMD_C_PIN;
  //    LL_GPIO_Init(CMD_D_PORT, &GPIO_InitStructure);
  //    if(PIN(cmdd) > 0){
  //       LL_GPIO_SetOutputPin(CMD_D_PORT, CMD_D_PIN);
  //    }
  //    else{
  //       LL_GPIO_ResetOutputPin(CMD_D_PORT, CMD_D_PIN);
  //    }
  // }
  // else{
  //    GPIO_InitStructure.Mode  = LL_GPIO_MODE_INPUT;
  //    GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_LOW;
  //    GPIO_InitStructure.Pull  = LL_GPIO_PULL_NO;
  //    GPIO_InitStructure.Pin = CMD_D_PIN;
  //    LL_GPIO_Init(CMD_D_PORT, &GPIO_InitStructure);
  //    LL_GPIO_ResetOutputPin(CMD_D_EN_PORT, CMD_D_EN_PIN);
  //    PIN(cmdd) = (CMD_D_PORT->IDR & CMD_D_PIN) > 0;
  // }

  if(PIN(cmdy) > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_6);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_6);

  if(PIN(cmdg) > 0)
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_7);
  else
    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_7);

  if(PIN(fan) > 0)
    LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3);
  else
    LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_3);

  if(PIN(fbsd) > 0)
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
  else
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  PIN(CRX) = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_0);
  PIN(CTX) = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_1);
  PIN(C12) = LL_GPIO_IsInputPinSet(CMD_A_PORT, CMD_A_PIN);
  PIN(C36) = LL_GPIO_IsInputPinSet(CMD_B_PORT, CMD_B_PIN);
  PIN(C54) = LL_GPIO_IsInputPinSet(CMD_C_PORT, CMD_C_PIN);
  PIN(C78) = LL_GPIO_IsInputPinSet(CMD_D_PORT, CMD_D_PIN);

  PIN(fb0a) = LL_GPIO_IsInputPinSet(FB0_A_PORT, FB0_A_PIN);
  PIN(fb0b) = LL_GPIO_IsInputPinSet(FB0_B_PORT, FB0_B_PIN);
  PIN(fb0z) = LL_GPIO_IsInputPinSet(FB0_Z_PORT, FB0_Z_PIN);

  PIN(fb1a) = LL_GPIO_IsInputPinSet(FB1_A_PORT, FB1_A_PIN);
  PIN(fb1b) = LL_GPIO_IsInputPinSet(FB1_B_PORT, FB1_B_PIN);
  PIN(fb1z) = LL_GPIO_IsInputPinSet(FB1_Z_PORT, FB1_Z_PIN);
}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = frt_func,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct io_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
