/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2015 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2015 Nico Stute <crinq@crinq.de>
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

//#include "stm32f4xx_conf.h"
#include "hal.h"
#include "setup.h"
#include "defines.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "f4_clock.h"
#include "f4_wdg.h"
#include "commands.h"
#include "hw/hw.h"

extern LL_RCC_ClocksTypeDef RCC_Clocks;

uint32_t hal_get_systick_value() {
  return (SysTick->VAL);
}

uint32_t hal_get_systick_reload() {
  return (SysTick->LOAD);
}

uint32_t hal_get_systick_freq() {
  return (RCC_Clocks.HCLK_Frequency);
}

volatile uint64_t systime = 0;
static uint32_t reset_flags;  // RCC_CSR reset flags of the last reset, see f4_clock.h

void SysTick_Handler(void) {
  systime++;
}

// HAL (USB, flash) timebase: the 1 kHz SysTick above, no HAL_Init()
uint32_t HAL_GetTick(void) {
  return (uint32_t)systime;
}

// encf req_sync: request word to send on the next frt timer edge, 0 = none.
// Written first thing in the highest priority interrupt, so the encoder
// latches at a fixed time after the timer update, whatever the rt is doing.
volatile uint16_t fb0_req_word;

//20kHz
void TIM_SLAVE_HANDLER(void) {
  if(fb0_req_word) {
    SPI3->DR     = fb0_req_word;
    fb0_req_word = 0;
  }
  LL_TIM_ClearFlag_UPDATE(TIM_SLAVE);
  hal_run_frt();
  if(LL_TIM_IsActiveFlag_UPDATE(TIM_SLAVE)) {
    hal_stop();
    hal.hal_state = FRT_TOO_LONG;
  }
}

//5 kHz interrupt for hal. at this point all ADCs have been sampled,
//see setup_res() in setup.c if you are interested in the magic behind this.
// fb0 encoder counter and A/B pins, latched as close to the sin/cos samples
// as the rt can get, for enc_fb. Reading them has no side effects, so this
// runs whichever fb0 component is loaded.
volatile uint32_t fb0_cnt_latch;
volatile uint32_t fb0_idr_latch;
// rt periods started, counted with the DMA TC flag clear (sserial ages its
// position by it, see there)
volatile uint32_t rt_tick;

void DMA2_Stream0_IRQHandler(void) {
  fb0_cnt_latch = FB0_ENC_TIM->CNT;
  fb0_idr_latch = FB0_A_PORT->IDR;
  // the frt (higher priority) reads rt_tick and the TC flag together
  __disable_irq();
  rt_tick++;
  LL_DMA_ClearFlag_TC0(DMA2);
  __enable_irq();
  hal_run_rt();
  if(LL_DMA_IsActiveFlag_TC0(DMA2)) {
    hal_stop();
    hal.hal_state = RT_TOO_LONG;
  }
}

// RM0090 13.8.1: an ADC overrun blocks the DMA requests and ignores further
// triggers until the ADC and DMA are set up again, so the rt (the DMA TC)
// would stop for good with nothing else noticing. Stop the hal and say so.
void ADC_IRQHandler(void) {
  if((ADC1->SR | ADC2->SR) & ADC_SR_OVR) {
    ADC1->CR1 &= ~ADC_CR1_OVRIE;
    ADC2->CR1 &= ~ADC_CR1_OVRIE;
    ADC1->SR = ~ADC_SR_OVR;
    ADC2->SR = ~ADC_SR_OVR;
    hal_stop();
    hal.hal_state = MISC_ERROR;
  }
}

// Jump to the ROM DFU bootloader from the app. The F4 bootloader on boards in
// the field starts the PLL and jumps to the ROM with it still running, so a
// reset-based entry only works with a new F4 bootloader. AN2606 wants clocks,
// PLL and interrupts back at reset before a jump to system memory.
void bootloader(char *ptr) {
  hal_stop();

  // AN2606: interrupts disabled and none pending. PRIMASK stays clear: the
  // ROM's USB DFU runs on interrupts and does not enable them.
  for(int i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;

  void (*SysMemBootJump)(void);
  volatile uint32_t addr = 0x1FFF0000;

  // LL_RCC_DeInit writes SW = HSI and clears PLLON back to back; PLLON can't
  // be cleared while the PLL still clocks the core (RM0090 6.2.6), and
  // LL_RCC_DeInit then waits forever for PLLRDY to drop. Finish the switch
  // first, as SystemInit does.
  LL_RCC_HSI_Enable();
  while(!LL_RCC_HSI_IsReady()) {
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }
  LL_RCC_DeInit();

  RCC->AHB1RSTR = 0x22E017FF;
  RCC->AHB1RSTR = 0;
  RCC->AHB2RSTR = 0xF1;
  RCC->AHB2RSTR = 0;
  RCC->AHB3RSTR = 0x1;
  RCC->AHB3RSTR = 0;
  RCC->APB1RSTR = 0xF6FEC9FF;
  RCC->APB1RSTR = 0;
  RCC->APB2RSTR = 0x4777933;
  RCC->APB2RSTR = 0;

  // AN2606: peripheral clocks off, enables back at their reset values
  RCC->AHB1ENR = 0x00100000;  // CCM data RAM only
  RCC->AHB2ENR = 0;
  RCC->AHB3ENR = 0;
  RCC->APB1ENR = 0;
  RCC->APB2ENR = 0;

  // No SYSCFG->MEMRMP = 1: it never took effect before SYSCFG got its clock,
  // and the jump works on the ROM's own vectors without the remap (AN2606
  // names the remap only for dual-bank parts).
  wdg_kick();  // the ROM DFU feeds a hardware watchdog from here on (AN2606 Table 73)
  SysMemBootJump = (void (*)(void))(*((uint32_t *)(addr + 4)));
  __set_MSP(*(uint32_t *)addr);
  SysMemBootJump();
}
COMMAND("bootloader", bootloader, "enter bootloader");

void nv_reset(char *ptr) {
  NVIC_SystemReset();
}
COMMAND("reset", nv_reset, "reset STMBL");

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
#ifdef __STM32F4xx_CMSIS_VERSION
  printf("F4 CMSIS   %i.%i.%i\n", __STM32F4xx_CMSIS_VERSION_MAIN, __STM32F4xx_CMSIS_VERSION_SUB1, __STM32F4xx_CMSIS_VERSION_SUB2);
#endif
#ifdef __STM32F3xx_HAL_VERSION
  printf("HAL lib... TODO: print version\n");
#endif
  printf("CPU ID     %lx %lx %lx\n",U_ID[0], U_ID[1], U_ID[2]);
  if(hse_failed) {
    printf("clock      HSE failed, running on HSI, hal not started\n");
  }
  printf("watchdog   %s\n", wdg_is_hw() ? "hardware IWDG (WDG_SW = 0)" : "off (WDG_SW = 1)");
  printf("reset      0x%08lx%s%s%s%s%s%s%s\n", reset_flags, reset_flags ? ":" : " (none: entered without a reset)",
         (reset_flags & RCC_CSR_LPWRRSTF) ? " low-power" : "", (reset_flags & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
         (reset_flags & RCC_CSR_IWDGRSTF) ? " IWDG" : "", (reset_flags & RCC_CSR_SFTRSTF) ? " software" : "",
         (reset_flags & RCC_CSR_PORRSTF) ? " power-on" : ((reset_flags & RCC_CSR_BORRSTF) ? " brown-out" : ""), (reset_flags & RCC_CSR_PINRSTF) ? " pin" : "");
  printf("size: %lu crc:%lx\n", version_info.image_size, version_info.image_crc);
  volatile const version_info_t *bt_version_info = (void *)0x08000188;
  printf("######## Bootloader info ########\n");
  printf(
      "%s v%i.%i.%i %s\n",
      bt_version_info->product_name,
      bt_version_info->major,
      bt_version_info->minor,
      bt_version_info->patch,
      bt_version_info->git_version);
  extern uint8_t _binary_obj_hvf3_hvf3_bin_start;
  extern uint8_t _binary_obj_hvf3_hvf3_bin_size;
  extern uint8_t _binary_obj_hvf3_hvf3_bin_end;
  volatile const version_info_t *hv_version_info = (void *)(&_binary_obj_hvf3_hvf3_bin_start + 0x188);
  printf("######## HV info ########\n");
  printf(
      "%s v%i.%i.%i %s\n",
      hv_version_info->product_name,
      hv_version_info->major,
      hv_version_info->minor,
      hv_version_info->patch,
      hv_version_info->git_version);
  // printf("Branch %s\n",bt_version_info->git_branch);
  // printf("Compiled %s %s ",bt_version_info->build_date, bt_version_info->build_time);
  // printf("by %s on %s\n",bt_version_info->build_user, bt_version_info->build_host);
  //

  printf("hv start:%p ,size:%p ,end%p \n", &_binary_obj_hvf3_hvf3_bin_start, &_binary_obj_hvf3_hvf3_bin_size, &_binary_obj_hvf3_hvf3_bin_end);
}

COMMAND("about", about, "show system infos");


void sleep(char *ptr) {
  float foo = 0;
  sscanf(ptr, " %f", &foo);
  printf("sleeping for %fs\n", foo);
  Wait((uint32_t)(foo * 1000));
  printf("wakeup\n");
}

COMMAND("sleep", sleep, "sleep [s]");

int main(void) {
  // Relocate interrupt vectors
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  // .ccm is NOLOAD, so the startup leaves it alone: zero it like .bss
  extern uint32_t _sccm, _eccm;
  for(uint32_t *p = &_sccm; p < &_eccm; p++) {
    *p = 0;
  }

  reset_flags = reset_flags_take();
  wdg_init();
  setup();
  hal_init(0.0002, 0.00005);
  // hal load comps
  load_comp(comp_by_name("term"));
  hal_parse("flashloadconf");
  hal_parse("loadconf");
  hal_parse("relink");
  if(hse_failed) {
    // on the HSI the rt runs 5x slow and the F3 link and USB cannot work;
    // leave the hal stopped rather than run it at the wrong rate
    hal.hal_state = MISC_ERROR;
  } else {
    hal_parse("start");
  }

  LL_TIM_EnableCounter(TIM_MASTER);
  LL_TIM_EnableIT_UPDATE(TIM_SLAVE);

  while(1)  //run non realtime stuff
  {
    hal_run_nrt();
    //cdc_poll();
    Wait(1);
  }
}

void Wait(uint32_t ms) {
  uint64_t t = systime + ms;
  while(t >= systime) {
    wdg_kick();  // the nrt kicks the watchdog, see f4_wdg.c
  }
}
