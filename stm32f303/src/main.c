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
#include "periph.h"

#include <math.h>
#include "defines.h"
#include "hal.h"
#include "angle.h"

#include "version.h"
#include "common.h"
#include "commands.h"
#include "f3hw.h"

#include "usbd_cdc_if.h"

volatile uint64_t systime = 0;

void SysTick_Handler(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */
  systime++;
  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

uint32_t systick_freq;

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (systick_freq);
}

void Error_Handler(void);

// TIM8 runs on its own: once the rt stops (overrun, MISC_ERROR, stop) the
// last compares stay latched with no current loop behind them. Nothing on
// that path reliably calls rt_stop, so the tick checks the state instead.
void bridge_off(void) {
  LL_TIM_DisableAllOutputs(TIM8);
#ifdef HV_EN_PIN
  LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
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
  LL_IWDG_Enable(IWDG);                               // start; from here only a reset stops it
  LL_IWDG_EnableWriteAccess(IWDG);                    // unlock PR and RLR
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);  // LSI / 4: 0.1 ms per count
  LL_IWDG_SetReloadCounter(IWDG, rlr);
  while(!LL_IWDG_IsReady(IWDG)) {
  }
  LL_IWDG_ReloadCounter(IWDG);
  __enable_irq();
  if(LL_IWDG_GetReloadCounter(IWDG) != rlr || LL_IWDG_GetPrescaler(IWDG) != LL_IWDG_PRESCALER_4) {
    Error_Handler();  // not armed as asked: stop here, the IWDG resets us
  }
}

void hal_reset_watchdog() {
  LL_IWDG_ReloadCounter(IWDG);
}

// Second shutdown path: a comparator break (BRK or BRK2, after the 0xC
// digital filter) has already cleared MOE in hardware. Raise HV_EN (IPM
// ITRIP) here, within about a microsecond, instead of waiting for io.c's
// next rt tick. The IPM then turns its gate driver off on its own and holds
// it off for at least 40 us. BIE is switched off so the still-set break
// flag can't re-enter; io.c's nrt re-arms it once the bridge is idle.
// io.c's rt keeps using MOE and the flags to report HV_OVERCURRENT_HW.
void TIM8_BRK_IRQHandler() {
  LL_TIM_DisableAllOutputs(TIM8);
#ifdef HV_EN_PIN
  LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
  LL_TIM_DisableIT_BRK(TIM8);
}

void TIM8_UP_IRQHandler() {
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
  LL_TIM_ClearFlag_UPDATE(TIM8);
  hal_run_rt();
  if(LL_TIM_IsActiveFlag_UPDATE(TIM8)) {
    hal_stop();
    hal.hal_state = RT_TOO_LONG;
  }
  if(hal.rt_state == RT_STOP) {
    bridge_off();
  }
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
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
#ifdef __STM32F3xx_CMSIS_VERSION
  printf("F3 CMSIS   %i.%i.%i\n", __STM32F3xx_CMSIS_VERSION_MAIN, __STM32F3xx_CMSIS_VERSION_SUB1, __STM32F3xx_CMSIS_VERSION_SUB2);
#endif
}

COMMAND("about", about, "show system infos");

void bootloader(char *ptr) {
#ifdef USB_DISCONNECT_PIN
  LL_GPIO_SetOutputPin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
  delay_ms(100);
#endif
  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0xDEADBEEF);
  NVIC_SystemReset();
}

COMMAND("bootloader", bootloader, "enter bootloader");

void reset(char *ptr) {
  NVIC_SystemReset();
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

  clock_init();
  systick_freq = SystemCoreClock;

  // RM0316 20.3.16: with the break filters on (BKF/BK2F 0xC), break handling
  // is only guaranteed with a fail-safe clock. CSS switches to HSI if the HSE
  // fails and also drives TIM8's break (9.2.7); NMI_Handler clears it.
  LL_RCC_HSE_EnableCSS();
  // RM0316 20.3.28: with the core halted by a debugger, stop TIM8 and let its
  // outputs go to the OSSI idle state instead of holding the last compares
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
  // a core lockup (fault inside a fault handler) breaks TIM8 in hardware
  // (SYSCFG_CFGR2 LOCKUP_LOCK, SYSCFG clocked in clock_init), whatever the
  // software is doing
  LL_SYSCFG_SetTIMBreakInputs(LL_SYSCFG_TIMBREAK_LOCKUP);
  /* Initialize all configured peripherals */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
#ifdef USB_DISCONNECT_PIN
  GPIO_InitStruct.Pin        = USB_DISCONNECT_PIN;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(USB_DISCONNECT_PORT, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
#endif

  tim8_init();
  adc_init();
  dac_init();

  //COMP1 in+ pa1 = W (ADC1_IN2)  in- pa4(dac1_ch1) out TIM8 BRK2
  comp_start(COMP1, LL_COMP_OUTPUT_TIM8_BKIN2);
  //COMP2 in+ pa7 = U (ADC2_IN4)  in- pa4(dac1_ch1) out TIM8 BRK2, like COMP1: BRK_ACTH has
  //no digital filter (RM0316 table 27 note), so U alone tripped on switching spikes
  comp_start(COMP2, LL_COMP_OUTPUT_TIM8_BKIN2);
  //COMP4 in+ pb0 = V (ADC3_IN12) in- pa4(dac1_ch1)  out TIM8 BRK
  comp_start(COMP4, LL_COMP_OUTPUT_TIM8_BKIN);
  // no hysteresis: tried low (RM0316 17.3.5) on Y, it holds the output through
  // the ringing after an edge and the break filter then trips 10-25 counts
  // earlier. Lock the three CSRs read-only until reset (17.3.4), as the RM
  // suggests for overcurrent protection; the threshold is the DAC, not the CSR.
  LL_COMP_Lock(COMP1);
  LL_COMP_Lock(COMP2);
  LL_COMP_Lock(COMP4);

  opamp_init();

#ifdef USB_CONNECT_PIN
  GPIO_InitStruct.Pin        = USB_CONNECT_PIN;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(USB_CONNECT_PORT, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(USB_CONNECT_PORT, USB_CONNECT_PIN);
#endif

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMA2);
  LL_RCC_EnableRTC();

  adc_calibrate();
  opamp_calibrate();

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);  // reset config: CRC-32, init 0xFFFFFFFF, no reversal

  //IO pins
  GPIO_InitStruct.Pin        = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  opamp_start();

  LL_TIM_OC_SetCompareCH1(TIM8, 0);
  LL_TIM_OC_SetCompareCH2(TIM8, 0);
  LL_TIM_OC_SetCompareCH3(TIM8, 0);

  // ADC1/2 and ADC3/4 in regular simultaneous dual mode, one DMA request per
  // pair (MDMA 10, 12 bit) that reads both results from the common CDR. CDR
  // holds the slave's result only in dual mode (RM0316 15.6.3, 15.3.29). The
  // sequences match rank for rank in length and sampling time, as the mode
  // needs. DUAL and MDMA only while the ADCs are disabled, and DMAEN/DMACFG
  // only with ADSTART = 0 (RM0316 15.5.4): all of it before the start, so the
  // first DMA word is always rank 1.
  extern volatile uint32_t adc_12_buf[ADC_SEQ_LEN];
  extern volatile uint32_t adc_34_buf[ADC_SEQ_LEN];
  adc_dma_init(DMA1, LL_DMA_CHANNEL_1, ADC12_COMMON, adc_12_buf);
  adc_dma_init(DMA2, LL_DMA_CHANNEL_5, ADC34_COMMON, adc_34_buf);

  adc_start();  // dual mode: enables all four, ADSTART on the masters only
  dac_start();
  tim8_start();

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
  hal_parse("load angle");
  hal_parse("load obs");

  hal_parse("ls0.rt_prio = 0.6");
  hal_parse("angle0.rt_prio = 0.7");
  hal_parse("io0.rt_prio = 1.0");
  hal_parse("dq0.rt_prio = 2.0");
  hal_parse("obs0.rt_prio = 2.5");  // after dq0, before curpid0: last tick's voltage
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
  hal_parse("idq0.pos = angle0.pos_v");  // the voltage angle, see ls.c v_lead
  hal_parse("idq0.mode = ls0.phase_mode");
  hal_parse("idq0.ext_sc = 0");  // own sincos at pos_v; dq0 keeps the sample angle
  hal_parse("dq0.pos = angle0.pos");
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
  hal_parse("curpid0.vel = angle0.vel");
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
  hal_parse("emf0.pos = angle0.pos");
  hal_parse("emf0.si = dq0.si");
  hal_parse("emf0.co = dq0.co");
  hal_parse("emf0.en = ls0.en");
  hal_parse("emf0.run = ls0.emf_run");
  hal_parse("emf0.sel = ls0.emf_sel");
  hal_parse("emf0.pp = ls0.emf_pp");
  hal_parse("ls0.emf_val = emf0.val");

  // flux observer. obs_mode 0: off, angle0 passes ls0.pos through. 1: shadow,
  // ls0.obs_err = obs angle minus the frame. 2: commutate from obs0.
  hal_parse("angle0.src = ls0.obs_src");
  hal_parse("angle0.pos_fb = ls0.pos");
  hal_parse("angle0.vel_fb = ls0.vel");
  hal_parse("angle0.v_lead = ls0.v_lead");
  hal_parse("angle0.pos_obs = obs0.pos_c");
  hal_parse("angle0.vel_obs = obs0.vel");
  hal_parse("obs0.en = ls0.obs_en");
  hal_parse("obs0.r = ls0.r");
  hal_parse("obs0.ld = ls0.l");
  hal_parse("obs0.lq = ls0.lq");
  hal_parse("obs0.bw = ls0.obs_bw");
  hal_parse("obs0.max_vel = 10000");
  hal_parse("obs0.id = dq0.d");
  hal_parse("obs0.iq = dq0.q");
  hal_parse("obs0.ud = curpid0.ud");
  hal_parse("obs0.uq = curpid0.uq");
  hal_parse("obs0.pos_ref = angle0.pos");
  hal_parse("obs0.vel_ref = angle0.vel");
  hal_parse("ls0.obs_err = obs0.pos_err");
  hal_parse("ls0.obs_vel = obs0.vel");

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
    delay_ms(1);
  }
}

// unrecoverable setup error: LED on, the IWDG (once armed) resets
void Error_Handler(void) {
  while(1) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
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
