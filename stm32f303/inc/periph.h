#pragma once

// F3 peripheral setup on the LL helpers (see periph.c).

#include "stm32f3xx.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_opamp.h"
#include "stm32f3xx_ll_comp.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_crc.h"
#include "stm32f3xx_ll_iwdg.h"
#include "stm32f3xx_ll_rtc.h"

void delay_ms(uint32_t ms);

// ms since boot from the 1 kHz SysTick
static inline uint32_t tick_ms(void) {
  extern volatile uint64_t systime;
  return (uint32_t)systime;
}

void clock_init(void);
void tim8_init(void);
void tim8_start(void);
void adc_init(void);
void adc_calibrate(void);
void adc_start(void);
void dac_init(void);
void dac_start(void);
void opamp_init(void);
void opamp_calibrate(void);
void opamp_start(void);
void comp_start(COMP_TypeDef *comp, uint32_t output);
void adc_dma_init(DMA_TypeDef *dma, uint32_t channel, ADC_Common_TypeDef *common, volatile uint32_t *buf);

// CRC-32 over 32-bit words, unit in its reset configuration, restarted from 0xFFFFFFFF
static inline uint32_t crc_calc(const uint32_t *buf, uint32_t len) {
  LL_CRC_ResetCRCCalculationUnit(CRC);
  for(uint32_t i = 0; i < len; i++) {
    LL_CRC_FeedData32(CRC, buf[i]);
  }
  return LL_CRC_ReadData32(CRC);
}
