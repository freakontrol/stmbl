#include "stm32f3xx_ll_gpio.h"

//stmbl
#define AREF 3.362  // analog reference voltage, measured on the X board
#define HV_EN_PIN LL_GPIO_PIN_15
#define HV_EN_PORT GPIOA
//fault pin cannot be used, as it is sometimes reset by the iram due to 15v ripple
#define HV_FAULT_PIN LL_GPIO_PIN_7
#define HV_FAULT_PORT GPIOB
#define HV_FAULT_POLARITY 0
#define LED_PIN LL_GPIO_PIN_8
#define LED_PORT GPIOA
#define BRK_PIN LL_GPIO_PIN_2
#define BRK_PORT GPIOB

#define VDIVUP 249000.0 * 2.0  //HV div pullup R1,R12
#define VDIVDOWN 3900.0        //HV div pulldown R2,R9
#define SHUNT 0.003            //shunt
#define SHUNT_PULLUP 15000.0
#define SHUNT_SERIE 470.0

#define PWM_U TIM8->CCR3
#define PWM_V TIM8->CCR2
#define PWM_W TIM8->CCR1

#define PWM_TIM_CLK 144000000.0  // TIM8 clock, prescaler 0, CKD div 1
// BDTR.DTG code, not a tick count: 196 = 0b110_00100 is (32 + 4) * 8 = 288
// ticks = 2.0us. DTG_TICKS decodes the four DTG ranges into the ticks ARR
// counts (valid while tim8 runs at CKD = DIV1).
#define PWM_DEADTIME 196
#define DTG_TICKS(v) ((v) < 0x80 ? (v) : (v) < 0xC0 ? ((64 + ((v)&0x3F)) * 2) : (v) < 0xE0 ? ((32 + ((v)&0x1F)) * 8) : ((32 + ((v)&0x1F)) * 16))
#define PWM_DEADTIME_TICKS DTG_TICKS(PWM_DEADTIME)
#define PWM_RES 4800

// ADC regular sequence per ADC pair: ADC_CUR_SAMPLES current samples, then
// one slow voltage (adc.c)
#define ADC_CUR_SAMPLES 3
#define ADC_SEQ_LEN (ADC_CUR_SAMPLES + 1)

// How long the phase current sample takes, from the TIM8 trigger at the
// counter extreme: ADC_CUR_SAMPLES conversions of 61.5 + 12.5 ADC clocks at
// 72 MHz (3.1 us for three), plus about 0.5 us for the sense node to ring
// down after the low side's own switching edge (scope, 2026-09-25). io.c
// uses it to tell which phase's low side was not on for the whole window.
// In timer ticks.
#define ADC_CUR_WINDOW_TICKS ((int32_t)((ADC_CUR_SAMPLES * 74.0 / 72e6 + 0.5e-6) * PWM_TIM_CLK))

#define ABS_MAX_TEMP 110.0
#define ABS_MAX_VOLT 400.0
#define ABS_MAX_CURRENT 35.0  // shunt measurement range (3 mOhm, +-35 A), not a module rating

//io board
//#define USB_CONNECT_PIN LL_GPIO_PIN_15
//#define USB_CONNECT_PORT GPIOB

/*
//otter
//TODO: swap v,w cur feedback
#define PWM_INVERT
#define AREF 3.3// analog reference voltage

#define VDIVUP 56000.0//HV div pullup R1,R12
#define VDIVDOWN 2000.0//HV div pulldown R2,R9
#define SHUNT 0.003//shunt
#define SHUNT_PULLUP 5100.0
#define SHUNT_SERIE 100.0

#define LED_Pin LL_GPIO_PIN_0
#define LED_GPIO_Port GPIOA

#define PWM_U TIM8->CCR1
#define PWM_V TIM8->CCR2
#define PWM_W TIM8->CCR3
 
//ottercontrol
#define USB_DISCONNECT_PIN LL_GPIO_PIN_13
#define USB_DISCONNECT_PORT GPIOC

#define PWM_DEADTIME 50
*/
