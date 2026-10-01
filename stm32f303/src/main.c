/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "tim.h"

#include <math.h>
#include "defines.h"
#include "hal.h"
#include "angle.h"

#include "version.h"
#include "common.h"
#include "commands.h"
#include "f3hw.h"

#include "usbd_cdc_if.h"
#ifdef USB_TERM
#include "usb_device.h"
#endif

volatile uint64_t systime = 0;

void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */
  systime++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

uint32_t systick_freq;
CRC_HandleTypeDef hcrc;

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (systick_freq);
}

void SystemClock_Config(void);
void Error_Handler(void);

// TIM8 runs on its own: once the rt stops (overrun, MISC_ERROR, stop) the
// last compares stay latched with no current loop behind them. Nothing on
// that path reliably calls rt_stop, so the tick checks the state instead.
void bridge_off(void) {
  TIM8->BDTR &= ~TIM_BDTR_MOE;
#ifdef HV_EN_PIN
  HAL_GPIO_WritePin(HV_EN_PORT, HV_EN_PIN, GPIO_PIN_SET);
#endif
}

// Independent watchdog on the 40 kHz LSI, kicked by hal.c at the end of every
// rt run while the nrt loop keeps coming round (HAL_WATCHDOG). It fires on an
// rt stop (overrun or error), a hang with irqs off, and an nrt stuck for 1 s;
// after the reset the pwm pins are inputs and HV_EN's pull-up holds the gates
// off.
void hal_init_watchdog(float time) {
  // RM0316 25.3.4: any other KR write between 0x5555 and the RLR write
  // re-locks PR/RLR, and the IWDG then runs at the 409 ms reset value. So the
  // reload is computed first and the sequence runs with interrupts off.
  uint32_t rlr = (uint32_t)CLAMP(time * 10000.0f, 1.0f, 4095.0f);
  __disable_irq();
  IWDG->KR  = 0xCCCC;  // start; from here only a reset stops it
  IWDG->KR  = 0x5555;  // unlock PR and RLR
  IWDG->PR  = 0;       // LSI / 4: 0.1 ms per count
  IWDG->RLR = rlr;
  while(IWDG->SR) {
  }
  IWDG->KR = 0xAAAA;
  __enable_irq();
  if(IWDG->RLR != rlr || IWDG->PR != 0) {
    Error_Handler();  // not armed as asked: stop here, the IWDG resets us
  }
}

void hal_reset_watchdog() {
  IWDG->KR = 0xAAAA;
}

// Second shutdown path: a comparator break (BRK or BRK2, after the 0xC
// digital filter) has already cleared MOE in hardware. Raise HV_EN (IPM
// ITRIP) here, within about a microsecond, instead of waiting for io.c's
// next rt tick. The IPM then turns its gate driver off on its own and holds
// it off for at least 40 us. BIE is switched off so the still-set break
// flag can't re-enter; io.c's nrt re-arms it once the bridge is idle.
// io.c's rt keeps using MOE and the flags to report HV_OVERCURRENT_HW.
void TIM8_BRK_IRQHandler() {
  TIM8->BDTR &= ~TIM_BDTR_MOE;
#ifdef HV_EN_PIN
  HV_EN_PORT->BSRR = HV_EN_PIN;
#endif
  TIM8->DIER &= ~TIM_DIER_BIE;
}

void TIM8_UP_IRQHandler() {
  GPIOA->BSRR |= GPIO_PIN_9;
  __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
  hal_run_rt();
  if(__HAL_TIM_GET_FLAG(&htim8, TIM_IT_UPDATE) == SET) {
    hal_stop();
    hal.hal_state = RT_TOO_LONG;
  }
  if(hal.rt_state == RT_STOP) {
    bridge_off();
  }
  GPIOA->BSRR |= GPIO_PIN_9 << 16;
}

void about(char *ptr) {
  printf("######## software info ########\n");
  printf(
      "%s v%i.%i.%i %s\n",
      version_info.product_name,
      version_info.major,
      version_info.minor,
      version_info.patch,
      version_info.git_version);
  printf("Branch %s\n", version_info.git_branch);
  printf("Compiled %s %s ", version_info.build_date, version_info.build_time);
  printf("by %s on %s\n", version_info.build_user, version_info.build_host);
  printf("GCC        %s\n", __VERSION__);
  printf("newlib     %s\n", _NEWLIB_VERSION);
#ifdef __CM4_CMSIS_VERSION
  printf("CMSIS      %i.%i\n", __CM4_CMSIS_VERSION_MAIN, __CM4_CMSIS_VERSION_SUB);
#endif
#ifdef __STM32F4XX_STDPERIPH_VERSION
  printf("StdPeriph  %i.%i.%i\n", __STM32F4XX_STDPERIPH_VERSION_MAIN, __STM32F4XX_STDPERIPH_VERSION_SUB1, __STM32F4XX_STDPERIPH_VERSION_SUB2);
#endif
#ifdef __STM32F3xx_HAL_VERSION
  printf("HAL lib... TODO: print version\n");
#endif
}

COMMAND("about", about, "show system infos");

void bootloader(char *ptr) {
#ifdef USB_DISCONNECT_PIN
  HAL_GPIO_WritePin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
#endif
  RTC->BKP0R = 0xDEADBEEF;
  NVIC_SystemReset();
}

COMMAND("bootloader", bootloader, "enter bootloader");

void reset(char *ptr) {
  HAL_NVIC_SystemReset();
}
COMMAND("reset", reset, "reset STMBL");

int main(void) {
  // copy the rt path into CCM RAM, the startup code only copies .data
  extern uint32_t _siccmram, _sccmram, _eccmram;
  for(uint32_t *src = &_siccmram, *dst = &_sccmram; dst < &_eccmram;) {
    *dst++ = *src++;
  }

  // Relocate interrupt vectors
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  systick_freq = HAL_RCC_GetHCLKFreq();
  // RM0316 20.3.16: with the break filters on (BKF/BK2F 0xC), break handling
  // is only guaranteed with a fail-safe clock. CSS switches to HSI if the HSE
  // fails and also drives TIM8's break (9.2.7); NMI_Handler clears it.
  HAL_RCC_EnableCSS();
  // RM0316 20.3.28: with the core halted by a debugger, stop TIM8 and let its
  // outputs go to the OSSI idle state instead of holding the last compares
  DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM8_STOP;
  // a core lockup (fault inside a fault handler) breaks TIM8 in hardware
  // (SYSCFG_CFGR2 LOCKUP_LOCK), whatever the software is doing
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->CFGR2 |= SYSCFG_CFGR2_LOCKUP_LOCK;
  /* Initialize all configured peripherals */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

#ifdef USB_DISCONNECT_PIN
  GPIO_InitStruct.Pin   = USB_DISCONNECT_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_DISCONNECT_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN, GPIO_PIN_RESET);
#endif

  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_DAC_Init();
  //COMP1 in+ pa1 = W (ADC1_IN2)  in- pa4(dac1_ch1) out TIM8 BRK2
  COMP1->CSR = COMP_CSR_COMPxINSEL_2 | COMP1_CSR_COMP1OUTSEL_2 | COMP_CSR_COMPxEN;
  //COMP2 in+ pa7 = U (ADC2_IN4)  in- pa4(dac1_ch1) out TIM8 BRK2, like COMP1: BRK_ACTH has
  //no digital filter (RM0316 table 27 note), so U alone tripped on switching spikes
  COMP2->CSR = COMP_CSR_COMPxINSEL_2 | COMP2_CSR_COMP2OUTSEL_2 | COMP_CSR_COMPxEN;
  //COMP4 in+ pb0 = V (ADC3_IN12) in- pa4(dac1_ch1)  out TIM8 BRK
  COMP4->CSR = COMP_CSR_COMPxINSEL_2 | COMP4_CSR_COMP4OUTSEL_0 | COMP4_CSR_COMP4OUTSEL_1 | COMP_CSR_COMPxEN;
  // no hysteresis: tried low (RM0316 17.3.5) on Y, it holds the output through
  // the ringing after an edge and the break filter then trips 10-25 counts
  // earlier. Lock the three CSRs read-only until reset (17.3.4), as the RM
  // suggests for overcurrent protection; the threshold is the DAC, not the CSR.
  COMP1->CSR |= COMP_CSR_COMPxLOCK;
  COMP2->CSR |= COMP_CSR_COMPxLOCK;
  COMP4->CSR |= COMP_CSR_COMPxLOCK;
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  // MX_USART1_UART_Init();

#ifdef USB_TERM
  MX_USB_DEVICE_Init();
#endif

#ifdef USB_CONNECT_PIN
  GPIO_InitStruct.Pin   = USB_CONNECT_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_CONNECT_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(USB_CONNECT_PORT, USB_CONNECT_PIN, GPIO_PIN_SET);
#endif

  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_RTC_ENABLE();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

  HAL_OPAMP_SelfCalibrate(&hopamp1);
  HAL_OPAMP_SelfCalibrate(&hopamp2);
  HAL_OPAMP_SelfCalibrate(&hopamp3);

  hcrc.Instance                     = CRC;
  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;

  __HAL_RCC_CRC_CLK_ENABLE();

  if(HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }

  //IO pins
  GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  if(HAL_OPAMP_Start(&hopamp1) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_OPAMP_Start(&hopamp2) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    Error_Handler();
  }

  htim8.Instance->CCR1 = 0;
  htim8.Instance->CCR2 = 0;
  htim8.Instance->CCR3 = 0;

  // ADC1/2 and ADC3/4 in regular simultaneous dual mode, one DMA request per
  // pair (MDMA 10, 12 bit) that reads both results from the common CDR. CDR
  // holds the slave's result only in dual mode (RM0316 15.6.3, 15.3.29). The
  // sequences match rank for rank in length and sampling time, as the mode
  // needs. DUAL and MDMA only while the ADCs are disabled, and DMAEN/DMACFG
  // only with ADSTART = 0 (RM0316 15.5.4): all of it before the start, so the
  // first DMA word is always rank 1.
  extern volatile uint32_t adc_12_buf[ADC_SEQ_LEN];
  extern volatile uint32_t adc_34_buf[ADC_SEQ_LEN];
  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1_Channel1->CPAR  = (uint32_t) & (ADC12_COMMON->CDR);
  DMA1_Channel1->CMAR  = (uint32_t)adc_12_buf;
  DMA1_Channel1->CNDTR = ADC_SEQ_LEN;
  DMA1_Channel1->CCR   = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_CIRC;
  DMA1_Channel1->CCR |= DMA_CCR_EN;
  DMA2_Channel5->CCR &= ~DMA_CCR_EN;
  DMA2_Channel5->CPAR  = (uint32_t) & (ADC34_COMMON->CDR);
  DMA2_Channel5->CMAR  = (uint32_t)adc_34_buf;
  DMA2_Channel5->CNDTR = ADC_SEQ_LEN;
  DMA2_Channel5->CCR   = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_CIRC;
  DMA2_Channel5->CCR |= DMA_CCR_EN;
  const uint32_t ccr = ADC_CCR_DUAL_1 | ADC_CCR_DUAL_2 | ADC_CCR_MDMA_1 | ADC_CCR_DMACFG;  // DUAL 00110
  ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~(ADC_CCR_DUAL | ADC_CCR_MDMA)) | ccr;
  ADC34_COMMON->CCR = (ADC34_COMMON->CCR & ~(ADC_CCR_DUAL | ADC_CCR_MDMA)) | ccr;

  HAL_ADC_Start(&hadc2);  // slaves first: in dual mode only enables them
  HAL_ADC_Start(&hadc4);
  HAL_ADC_Start(&hadc1);  // masters: enable and ADSTART, the pair follows T8_TRGO
  HAL_ADC_Start(&hadc3);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  if(HAL_TIM_Base_Start_IT(&htim8) != HAL_OK) {
    Error_Handler();
  }
#ifndef PWM_INVERT
  TIM8->RCR = 1;  //uptate event foo
#endif
  if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if(HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  hal_init(1.0 / 15000.0, 0.0);
  // hal load comps
  hal_parse("debug_level 1");
  hal_parse("load term");
  // load_comp(comp_by_name("sim"));
  hal_parse("load io");
  hal_parse("load ls");
  hal_parse("load dq");
  hal_parse("load idq");
  hal_parse("load hv");
  hal_parse("load curpid");
  hal_parse("load emf");

  hal_parse("ls0.rt_prio = 0.6");
  hal_parse("io0.rt_prio = 1.0");
  hal_parse("dq0.rt_prio = 2.0");
  hal_parse("curpid0.rt_prio = 3.0");
  hal_parse("idq0.rt_prio = 4.0");
  hal_parse("hv0.rt_prio = 6.0");
  hal_parse("emf0.rt_prio = 7.0");

  hal_parse("term0.send_step = 0.0");

  //link LS
  hal_parse("ls0.mot_temp = io0.mot_temp");
  hal_parse("ls0.dc_volt = io0.udc");
  hal_parse("ls0.hv_temp = io0.hv_temp");
  hal_parse("ls0.fault_in = io0.fault");
  hal_parse("io0.led = ls0.fault");
  hal_parse("curpid0.id_cmd = ls0.d_cmd");
  hal_parse("curpid0.iq_cmd = ls0.q_cmd");
  hal_parse("idq0.pos = ls0.pos_v");  // the voltage angle, see ls.c
  hal_parse("idq0.mode = ls0.phase_mode");
  hal_parse("idq0.ext_sc = 0");  // own sincos at pos_v; dq0 keeps the sample angle
  hal_parse("dq0.pos = ls0.pos");
  hal_parse("dq0.mode = ls0.phase_mode");
  hal_parse("io0.hv_en = ls0.en");
  hal_parse("io0.sbrake = ls0.sbrake");
  hal_parse("hv0.sbrake = io0.sbrake_on");
  hal_parse("io0.dac = ls0.dac");

  //ADC TEST
  hal_parse("hv0.udc = io0.udc_duty");
  hal_parse("ls0.duty_max = hv0.duty_max");
  hal_parse("dq0.u = io0.iu");
  hal_parse("dq0.v = io0.iv");
  hal_parse("dq0.w = io0.iw");

  hal_parse("hv0.u = idq0.u");  // hv0 adds the dead-time comp, then the SVM offset
  hal_parse("hv0.v = idq0.v");
  hal_parse("hv0.w = idq0.w");
  hal_parse("hv0.id_fb = dq0.d");  // volt-mode dead-time reference
  hal_parse("hv0.iq_fb = dq0.q");
  hal_parse("curpid0.id_fb = dq0.d");
  hal_parse("curpid0.iq_fb = dq0.q");
  hal_parse("ls0.id_fb = dq0.d");
  hal_parse("ls0.iq_fb = dq0.q");
  hal_parse("ls0.ud_fb = curpid0.ud");
  hal_parse("ls0.uq_fb = curpid0.uq");
  hal_parse("ls0.y = dq0.y");
  hal_parse("ls0.u_fb = io0.u");
  hal_parse("ls0.v_fb = io0.v");
  hal_parse("ls0.w_fb = io0.w");
  hal_parse("idq0.d = curpid0.ud");
  hal_parse("idq0.q = curpid0.uq");
  hal_parse("curpid0.r = ls0.r");
  hal_parse("curpid0.ld = ls0.l");
  hal_parse("curpid0.lq = ls0.lq");  // ls0.lq falls back to ls0.l
  hal_parse("curpid0.psi = ls0.psi");
  hal_parse("curpid0.cur_bw = ls0.cur_bw");
  hal_parse("curpid0.ff = ls0.cur_ff");
  hal_parse("curpid0.kind = ls0.cur_ind");
  hal_parse("curpid0.max_cur = ls0.max_cur");
  hal_parse("io0.max_cur = ls0.max_cur");
  hal_parse("curpid0.pwm_volt = ls0.pwm_volt");
  hal_parse("curpid0.vel = ls0.vel");
  hal_parse("curpid0.en = ls0.en");
  hal_parse("curpid0.cmd_mode = ls0.cmd_mode");
  hal_parse("hv0.arr = ls0.arr");
  hal_parse("hv0.drop_k = ls0.drop_k");
  hal_parse("hv0.drop_knee = ls0.drop_knee");
  hal_parse("io0.ignore_fault_pin = ls0.ignore_fault_pin");
  // hal_parse("load sensorless");
  // hal_parse("sensorless0.rt_prio = 7");
  // hal_parse("sensorless0.r = ls0.r");
  // hal_parse("sensorless0.l = ls0.l");
  // hal_parse("sensorless0.id = dq0.d");
  // hal_parse("sensorless0.iq = dq0.q");
  // hal_parse("sensorless0.ud = curpid0.ud");
  // hal_parse("sensorless0.uq = curpid0.uq");
  // dead time compensation sign from the commanded current, not from io0
  hal_parse("hv0.d_cmd = ls0.d_cmd");
  hal_parse("hv0.q_cmd = ls0.q_cmd");
  hal_parse("hv0.si = idq0.si_out");  // dead-time reference on the voltage angle
  hal_parse("hv0.co = idq0.co_out");
  hal_parse("hv0.cmd_mode = ls0.cmd_mode");
  hal_parse("hv0.phase_mode = ls0.phase_mode");

  // per pole back emf on a bridge off coast, read back by idpmsm's psi test
  hal_parse("emf0.u = io0.ur");
  hal_parse("emf0.v = io0.vr");
  hal_parse("emf0.w = io0.wr");
  hal_parse("emf0.pos = ls0.pos");
  hal_parse("emf0.si = dq0.si");
  hal_parse("emf0.co = dq0.co");
  hal_parse("emf0.en = ls0.en");
  hal_parse("emf0.run = ls0.emf_run");
  hal_parse("emf0.sel = ls0.emf_sel");
  hal_parse("emf0.pp = ls0.emf_pp");
  hal_parse("ls0.emf_val = emf0.val");

  hal_parse("debug_level 0");

  // hal parse config
  // hal_init_nrt();
  // error foo
  // armed before the rt starts, so a hang in the very first rt tick resets
  // too, and no rt kick can land inside the key sequence
  hal_init_watchdog(0.005);
  hal_start();

  while(1) {
    hal_run_nrt();
    cdc_poll();
    HAL_Delay(1);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  //TODO: usb optional
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34 | RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection  = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection  = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim8ClockSelection   = RCC_TIM8CLK_PLLCLK;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);  // same level as the rt, below the break
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
  while(1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif
