// F3 peripheral setup on the LL helpers. Ported from the Cube HAL V1.5.8 init
// code and its CubeMX settings; the registers end up as they did there (fields
// it wrote to their reset value are left at reset). Raw register access only
// where LL has no helper.

// Boot-time setup only, nothing here runs in the rt: size over speed, as the
// inlined LL setters at the project's -O3 add ~1.5 KB of flash
#pragma GCC optimize("Os")

#include "periph.h"
#include "f3hw.h"

extern volatile uint64_t systime;

// Busy wait of at least ms full SysTick periods (the partial current one is not counted)
void delay_ms(uint32_t ms) {
  uint64_t start = systime;
  while(systime - start < (uint64_t)ms + 1) {
  }
}

static void gpio_analog(GPIO_TypeDef *port, uint32_t pins) {
  LL_GPIO_Init(port, &(LL_GPIO_InitTypeDef){.Pin = pins, .Mode = LL_GPIO_MODE_ANALOG, .Pull = LL_GPIO_PULL_NO});
}

static void gpio_af(GPIO_TypeDef *port, uint32_t pins, uint32_t af) {
  LL_GPIO_Init(port, &(LL_GPIO_InitTypeDef){.Pin = pins, .Mode = LL_GPIO_MODE_ALTERNATE, .Speed = LL_GPIO_SPEED_FREQ_LOW, .OutputType = LL_GPIO_OUTPUT_PUSHPULL, .Pull = LL_GPIO_PULL_NO, .Alternate = af});
}

// HSE 8 MHz * 9 = 72 MHz SYSCLK/HCLK, APB1 36 MHz, APB2 72 MHz,
// TIM8 and ADCs from the PLL (TIM8 144 MHz), USART3 from SYSCLK, RTC from LSI.
// SysTick 1 kHz, priority 14.
void clock_init(void) {
  LL_FLASH_EnablePrefetch();
  NVIC_SetPriorityGrouping(3);  // 4 bits preemption, no subpriority
  // SYSCFG on for the OPAMPs and comparators, fault handlers at the top priority
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  NVIC_SetPriority(MemoryManagement_IRQn, 0);
  NVIC_SetPriority(BusFault_IRQn, 0);
  NVIC_SetPriority(UsageFault_IRQn, 0);
  NVIC_SetPriority(SVCall_IRQn, 0);
  NVIC_SetPriority(DebugMonitor_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0);

  // oscillators
  LL_RCC_HSE_Enable();
  while(!LL_RCC_HSE_IsReady()) {
  }
  LL_RCC_HSI_Enable();
  while(!LL_RCC_HSI_IsReady()) {
  }
  LL_RCC_LSI_Enable();
  while(!LL_RCC_LSI_IsReady()) {
  }
  // f3_boot jumps here with SYSCLK on the PLL and the CubeF3 SystemInit no
  // longer resets RCC. PLLON can't be cleared while the PLL is SYSCLK, so
  // move SYSCLK to HSI first or PLLRDY never drops.
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }
  LL_RCC_PLL_Disable();
  while(LL_RCC_PLL_IsReady()) {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();
  while(!LL_RCC_PLL_IsReady()) {
  }

  // bus clocks: flash latency for 72 MHz first, APB dividers at their maximum
  // while SYSCLK switches to the PLL so no bus is ever overclocked, then the
  // final dividers
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_16);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  SystemCoreClockUpdate();

  // peripheral clock sources. The RTC source can only change after a backup
  // domain reset, which keeps BDCR's other settings.
  uint32_t pwr_was_off = !LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();  // stays set, RTC BKP0R is written by the bootloader command
  while(!LL_PWR_IsEnabledBkUpAccess()) {
  }
  uint32_t rtcsel = LL_RCC_GetRTCClockSource();
  if(rtcsel != LL_RCC_RTC_CLKSOURCE_NONE && rtcsel != LL_RCC_RTC_CLKSOURCE_LSI) {
    uint32_t bdcr = LL_RCC_ReadReg(BDCR) & ~RCC_BDCR_RTCSEL;
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_WriteReg(BDCR, bdcr);
  }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  if(pwr_was_off) {
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC34_CLKSRC_PLL_DIV_1);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM8_CLKSOURCE_PLL);

  SysTick_Config(SystemCoreClock / 1000);  // HCLK source
  NVIC_SetPriority(SysTick_IRQn, 14);       // below TIM8 (rt 1, break 0): its handler runs from flash and must not delay the rt
}

// Center aligned PWM on TIM8 CH1-3 with complementary outputs, dead time and
// both break inputs, TRGO on update for the ADCs. Outputs stay off (MOE and
// CCxE clear) until tim8_start().
void tim8_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  // the break interrupt must preempt the rt (TIM8_UP), so it gets the
  // highest priority and the rt moves one level down
  NVIC_SetPriority(TIM8_BRK_IRQn, 0);
  NVIC_EnableIRQ(TIM8_BRK_IRQn);
  NVIC_SetPriority(TIM8_UP_IRQn, 1);
  NVIC_EnableIRQ(TIM8_UP_IRQn);

  // time base, center aligned mode 3, ARR preloaded: hv0 rewrites ARR every
  // tick for the phase lock, and preloaded it changes at the next UEV along
  // with the compares (RM0316 20.3.1). The update event loads ARR/PSC/RCR;
  // URS is set around it so it raises no UIF, and cleared again after.
  LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);
  LL_TIM_SetClockDivision(TIM8, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_EnableARRPreload(TIM8);
  LL_TIM_SetAutoReload(TIM8, PWM_RES);
  LL_TIM_SetPrescaler(TIM8, 0);
#ifdef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);
#else
  LL_TIM_SetRepetitionCounter(TIM8, 0);
#endif
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_COUNTER);
  LL_TIM_GenerateEvent_UPDATE(TIM8);
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_REGULAR);

  // internal clock, TRGO = update, no master/slave
  LL_TIM_SetClockSource(TIM8, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_SetTriggerInput(TIM8, LL_TIM_TS_ITR0);
  LL_TIM_ConfigETR(TIM8, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_DisableMasterSlaveMode(TIM8);
  LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_RESET);
  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_UPDATE);

  // CH1-3 PWM mode 1 with preload, all six outputs off, active high, idle
  // low, compare 0. Setters rather than LL_TIM_OC_Init(), which handles all
  // six channels and costs ~800 B of flash.
  const uint32_t chs[]  = {LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3};
  const uint32_t chns[] = {LL_TIM_CHANNEL_CH1N, LL_TIM_CHANNEL_CH2N, LL_TIM_CHANNEL_CH3N};
  for(int i = 0; i < 3; i++) {
    LL_TIM_CC_DisableChannel(TIM8, chs[i] | chns[i]);
    LL_TIM_OC_SetPolarity(TIM8, chs[i], LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM8, chns[i], LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetIdleState(TIM8, chs[i], LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetIdleState(TIM8, chns[i], LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetMode(TIM8, chs[i], LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIM8, chs[i]);
    LL_TIM_OC_DisableFast(TIM8, chs[i]);
  }
  LL_TIM_OC_SetCompareCH1(TIM8, 0);
  LL_TIM_OC_SetCompareCH2(TIM8, 0);
  LL_TIM_OC_SetCompareCH3(TIM8, 0);

  // dead time, BRK (COMP4, V) and BRK2 (COMP1 W, COMP2 U) active high, no
  // automatic output. Filter 0xC: fDTS/16, N = 8, 0.89 us (0xF was 1.78 us).
  // The IM06B50GC1 allows about 3 us from overcurrent to off, 1.43 us of it
  // inside the module after ITRIP. On X (C38 3.3 uF, sense caps) 0xC trips
  // 3-5 A earlier than 0xF at the same hv0.dac from switching ringing, so the
  // dac goes up: 0xC at 255 trips where 0xF did at 215 (19.5-21 A); 0xA
  // tripped at 1 A at 150. RM0316 20.3.16: "BRK2 must only be used with OSSR =
  // OSSI = 1" (outputs to their idle level, all switches off, when MOE
  // drops), and BK2P and BK2E must not be set in the same BDTR write, so
  // BK2E follows on its own.
  LL_TIM_OC_SetDeadTime(TIM8, PWM_DEADTIME);
  LL_TIM_SetOffStates(TIM8, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
  LL_TIM_DisableAutomaticOutput(TIM8);
  LL_TIM_ConfigBRK(TIM8, LL_TIM_BREAK_POLARITY_HIGH, LL_TIM_BREAK_FILTER_FDIV16_N8);
  LL_TIM_EnableBRK(TIM8);
  LL_TIM_ConfigBRK2(TIM8, LL_TIM_BREAK2_POLARITY_HIGH, LL_TIM_BREAK2_FILTER_FDIV16_N8);
  LL_TIM_EnableBRK2(TIM8);
  __DSB();  // BK2E takes an APB cycle to act
  // LOCK level 1 (RM0316 20.3.16/20.4.21): DTG, BKE/BKP/BKF, BK2E/BK2P/BK2F,
  // AOE and OISx are read-only until reset, so no read-modify-write of BDTR
  // can change the break setup. MOE, which io0 toggles, stays writable. LL
  // has no LOCK setter outside LL_TIM_BDTR_Init().
  SET_BIT(TIM8->BDTR, LL_TIM_LOCKLEVEL_1);
  for(volatile int i = 0; i < 4; i++) {  // "wait 4 timer clocks" before B2IF
  }
  // Clears only what the setup itself raised: the comparators come up later
  // with the DAC at 0 V and set BIF/B2IF again; the enable path handles that.
  LL_TIM_ClearFlag_BRK(TIM8);
  LL_TIM_ClearFlag_BRK2(TIM8);

  // PB3 CH1N, PB4 CH2N, PB5 CH3N, PB6 CH1, PB8 CH2, PB9 CH3
  gpio_af(GPIOB, LL_GPIO_PIN_3 | LL_GPIO_PIN_4, LL_GPIO_AF_4);
  gpio_af(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_3);
  gpio_af(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_5);
  gpio_af(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_9, LL_GPIO_AF_10);
}

// Update interrupt and counter first, then the six outputs (MOE with the first
// channel, kept from the HAL start sequence the board was validated with).
void tim8_start(void) {
  LL_TIM_EnableIT_UPDATE(TIM8);
  LL_TIM_EnableCounter(TIM8);
#ifndef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);  //uptate event foo
#endif
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIM8);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
}

static const uint32_t adc_ranks[] = {0, LL_ADC_REG_RANK_1, LL_ADC_REG_RANK_2, LL_ADC_REG_RANK_3, LL_ADC_REG_RANK_4, LL_ADC_REG_RANK_5, LL_ADC_REG_RANK_6};

// One regular rank: channel, sampling time, single ended
__attribute__((noinline)) static void adc_rank(ADC_TypeDef *adc, uint32_t rank, uint32_t chan, uint32_t smp) {
  LL_ADC_REG_SetSequencerRanks(adc, adc_ranks[rank], chan);
  LL_ADC_SetChannelSamplingTime(adc, chan, smp);
  LL_ADC_SetChannelSingleDiff(adc, chan, LL_ADC_SINGLE_ENDED);
}

// Regulator on, 12 bit right aligned, overrun overwrites, ADC_SEQ_LEN
// conversion scan started by TIM8 TRGO (rising edge). In dual mode the
// slave's trigger settings are not used.
static void adc_setup(ADC_TypeDef *adc, ADC_Common_TypeDef *common, uint32_t trigger) {
  if(!LL_ADC_IsInternalRegulatorEnabled(adc)) {
    LL_ADC_EnableInternalRegulator(adc);
    for(__IO uint32_t wait = LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / 1000000U); wait; wait--) {
    }
  }
  LL_ADC_SetCommonClock(common, LL_ADC_CLOCK_ASYNC_DIV1);  // from the PLL, RCC prescaler 1
  LL_ADC_SetResolution(adc, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_SetLowPowerMode(adc, LL_ADC_LP_MODE_NONE);  // AUTDLY off
  LL_ADC_REG_SetContinuousMode(adc, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetSequencerDiscont(adc, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  LL_ADC_REG_SetOverrun(adc, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_NONE);  // the pair's DMA is in the common CCR
  // on F3 SetTriggerSource writes EXTSEL and EXTEN; the edge is set again to not depend on that
  LL_ADC_REG_SetTriggerSource(adc, trigger);
  LL_ADC_REG_SetTriggerEdge(adc, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetSequencerLength(adc, (ADC_SEQ_LEN - 1U) << ADC_SQR1_L_Pos);  // ADC_SEQ_LEN ranks
}


// ADC1-4 with their analog pins. Ranks 1-3
// sample the PGA outputs for 61.5 cycles = 854 ns, over the 400 ns DS9118
// table 77 asks for an OPAMP output; rank 4 is the slow voltage channel.
// The pairs run in dual simultaneous mode, so ADC4 follows the same ranks.
void adc_init(void) {
  // ADC1: iw (ch3, opamp1) x3, uw (ch4)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  gpio_analog(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
  adc_setup(ADC1, ADC12_COMMON, LL_ADC_REG_TRIG_EXT_TIM8_TRGO_ADC12);
  for(uint32_t r = 1; r <= ADC_CUR_SAMPLES; r++) {
    adc_rank(ADC1, r, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  }
  adc_rank(ADC1, ADC_SEQ_LEN, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC2: iu (ch3, opamp2) x3, uv (ch2)
  gpio_analog(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
  adc_setup(ADC2, ADC12_COMMON, LL_ADC_REG_TRIG_EXT_TIM8_TRGO_ADC12);
  for(uint32_t r = 1; r <= ADC_CUR_SAMPLES; r++) {
    adc_rank(ADC2, r, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  }
  adc_rank(ADC2, ADC_SEQ_LEN, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC3: iv (ch1, opamp3) x3, uu (ch5)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC34);
  gpio_analog(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_13);
  adc_setup(ADC3, ADC34_COMMON, LL_ADC_REG_TRIG_EXT_TIM8_TRGO__ADC34);
  for(uint32_t r = 1; r <= ADC_CUR_SAMPLES; r++) {
    adc_rank(ADC3, r, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  }
  adc_rank(ADC3, ADC_SEQ_LEN, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_181CYCLES_5);

  // ADC4: hv_temp (ch4) x2, mot_temp (ch5), hv (ch3)
  gpio_analog(GPIOB, LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
  adc_setup(ADC4, ADC34_COMMON, LL_ADC_REG_TRIG_EXT_TIM8_TRGO__ADC34);
  adc_rank(ADC4, 1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  adc_rank(ADC4, 2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  adc_rank(ADC4, 3, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_61CYCLES_5);
  adc_rank(ADC4, ADC_SEQ_LEN, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_181CYCLES_5);
}

// Single ended calibration; the ADCs must be disabled while it runs
void adc_calibrate(void) {
  ADC_TypeDef *const adcs[] = {ADC1, ADC2, ADC3, ADC4};
  for(int i = 0; i < 4; i++) {
    LL_ADC_StartCalibration(adcs[i], LL_ADC_SINGLE_ENDED);
    while(LL_ADC_IsCalibrationOnGoing(adcs[i])) {
    }
  }
}

// Enable all four, slaves first, then arm the masters' regular group for the
// TIM8 trigger. In dual simultaneous mode (DUAL/MDMA set in main.c while the
// ADCs were still disabled) the slaves convert with their master.
void adc_start(void) {
  ADC_TypeDef *const adcs[] = {ADC2, ADC4, ADC1, ADC3};
  for(int i = 0; i < 4; i++) {
    if(!LL_ADC_IsEnabled(adcs[i])) {
      LL_ADC_Enable(adcs[i]);
      while(!LL_ADC_IsActiveFlag_ADRDY(adcs[i])) {
      }
    }
    adcs[i]->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
  }
  LL_ADC_REG_StartConversion(ADC1);
  LL_ADC_REG_StartConversion(ADC3);
}

// DAC1 CH1 on PA4, no trigger. Output buffer off: its range is 0.5 mV to
// VDDA (DS9118 table 75) against 0.2 V minimum with the buffer, and the
// comparator thresholds sit at 0.1-0.3 V.
void dac_init(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  gpio_analog(GPIOA, LL_GPIO_PIN_4);
  // no trigger, no wave; TSEL1 and MAMP1 stay at their reset 0
  LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_SetWaveAutoGeneration(DAC1, LL_DAC_CHANNEL_1, LL_DAC_WAVE_AUTO_GENERATION_NONE);
  LL_DAC_SetOutputBuffer(DAC1, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_DISABLE);
}

void dac_start(void) {
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0);
}

static OPAMP_TypeDef *const opamps[] = {OPAMP1, OPAMP2, OPAMP3};

// PGA gain 16 on VP0, factory trim, left disabled until opamp_start()
void opamp_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  gpio_analog(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_2);  // OPAMP1 VINP, VOUT
  gpio_analog(GPIOA, LL_GPIO_PIN_6 | LL_GPIO_PIN_7);  // OPAMP2 VOUT, VINP
  gpio_analog(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1);  // OPAMP3 VINP, VOUT
  for(int i = 0; i < 3; i++) {
    // the secondary inputs (VPSSEL/VMSSEL) and FORCEVP stay at their reset 0
    LL_OPAMP_SetFunctionalMode(opamps[i], LL_OPAMP_MODE_PGA);
    LL_OPAMP_SetPGAGain(opamps[i], LL_OPAMP_PGA_GAIN_16);
    LL_OPAMP_SetInputNonInverting(opamps[i], LL_OPAMP_INPUT_NONINVERT_IO0);
    LL_OPAMP_SetInputsMuxMode(opamps[i], LL_OPAMP_INPUT_MUX_DISABLE);
    LL_OPAMP_SetTrimmingMode(opamps[i], LL_OPAMP_TRIMMING_FACTORY);
    LL_OPAMP_SetTrimmingValue(opamps[i], LL_OPAMP_TRIMMING_NMOS, 0);
    LL_OPAMP_SetTrimmingValue(opamps[i], LL_OPAMP_TRIMMING_PMOS, 0);
  }
}

// Binary search of one trim field against OUTCAL, the algorithm of
// HAL_OPAMP_SelfCalibrate(): 4 halving steps from 16, 2 ms settling each, +1 if
// OUTCAL is still set
static uint32_t opamp_trim(OPAMP_TypeDef *op, uint32_t pair) {
  uint32_t trim  = 16U;
  uint32_t delta = 8U;
  while(delta != 0U) {
    LL_OPAMP_SetTrimmingValue(op, pair, trim);
    delay_ms(2);
    if(LL_OPAMP_IsCalibrationOutputSet(op)) {
      trim += delta;
    } else {
      trim -= delta;
    }
    delta >>= 1U;
  }
  LL_OPAMP_SetTrimmingValue(op, pair, trim);
  delay_ms(2);
  if(LL_OPAMP_IsCalibrationOutputSet(op)) {
    trim++;
    LL_OPAMP_SetTrimmingValue(op, pair, trim);
  }
  return trim;
}

// User trim from self calibration, OPAMPs left disabled. FORCEVP has no LL
// helper on F3.
void opamp_calibrate(void) {
  for(int i = 0; i < 3; i++) {
    OPAMP_TypeDef *op = opamps[i];
    SET_BIT(op->CSR, OPAMP_CSR_FORCEVP);
    LL_OPAMP_SetTrimmingMode(op, LL_OPAMP_TRIMMING_USER);
    LL_OPAMP_SetMode(op, LL_OPAMP_MODE_CALIBRATION);
    LL_OPAMP_SetCalibrationSelection(op, LL_OPAMP_TRIMMING_NMOS);  // 90% VDDA, NMOS
    LL_OPAMP_Enable(op);
    uint32_t n = opamp_trim(op, LL_OPAMP_TRIMMING_NMOS);
    LL_OPAMP_SetCalibrationSelection(op, LL_OPAMP_TRIMMING_PMOS);  // 10% VDDA, PMOS
    uint32_t p = opamp_trim(op, LL_OPAMP_TRIMMING_PMOS);
    LL_OPAMP_SetMode(op, LL_OPAMP_MODE_FUNCTIONAL);
    LL_OPAMP_Disable(op);
    CLEAR_BIT(op->CSR, OPAMP_CSR_FORCEVP);
    LL_OPAMP_SetTrimmingValue(op, LL_OPAMP_TRIMMING_PMOS, p);
    LL_OPAMP_SetTrimmingValue(op, LL_OPAMP_TRIMMING_NMOS, n);
  }
}

void opamp_start(void) {
  for(int i = 0; i < 3; i++) {
    LL_OPAMP_Enable(opamps[i]);
  }
}

// Overcurrent comparator: in- from DAC1 CH1 (the trip threshold), in+ fixed
// to its phase's sense node, output to a TIM8 break input, no hysteresis
void comp_start(COMP_TypeDef *comp, uint32_t output) {
  LL_COMP_SetInputMinus(comp, LL_COMP_INPUT_MINUS_DAC1_CH1);
  LL_COMP_SetOutputSelection(comp, output);
  LL_COMP_Enable(comp);
}

// Circular DMA of one dual mode ADC pair: ADC_SEQ_LEN words from the common
// CDR (master result low, slave high), then DUAL 00110 (regular
// simultaneous) and MDMA 10 with DMACFG (12 bit, unlimited requests).
void adc_dma_init(DMA_TypeDef *dma, uint32_t channel, ADC_Common_TypeDef *common, volatile uint32_t *buf) {
  LL_DMA_DisableChannel(dma, channel);
  LL_DMA_ConfigAddresses(dma, channel, (uint32_t)&common->CDR, (uint32_t)buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(dma, channel, ADC_SEQ_LEN);
  LL_DMA_ConfigTransfer(dma, channel,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_WORD | LL_DMA_MDATAALIGN_WORD | LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_EnableChannel(dma, channel);
  LL_ADC_SetMultimode(common, LL_ADC_MULTI_DUAL_REG_SIMULT);
  LL_ADC_SetMultiDMATransfer(common, LL_ADC_MULTI_REG_DMA_UNLMT_RES12_10B);
}
