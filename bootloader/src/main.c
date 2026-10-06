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

#include "stm32f4xx_conf.h"
#include "f4_clock.h"
#include "version.h"

#if __GNUC__ < 5
#error gcc to old (< 5.0)
#endif

#define APP_START 0x08010000
#define APP_END 0x08100000
#define APP_RANGE_VALID(a, s) (!(((a) | (s)) & 3) && (a) >= APP_START && ((a) + (s)) <= APP_END)
#define VERSION_INFO_OFFSET 0x188
const version_info_t *app_info = (void *)(APP_START + VERSION_INFO_OFFSET);

static int app_ok(void) {
  if(!APP_RANGE_VALID(APP_START, app_info->image_size)) {
    return 0;
  }
  CRC->CR      = CRC_CR_RESET;
  uint32_t crc = crc_calc_block((const uint32_t *)APP_START, app_info->image_size / 4);

  if(crc != 0) {
    return 0;
  }

  return 1;
}

int main(void) {
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;
  clock_init();  // the ROM bootloader and the app are entered at 168 MHz

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_CRC);
  // PA13 input with pull up
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
  uint32_t pin = !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_13);
  LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_GPIOA);  // reset gpio a
  LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  void (*SysMemBootJump)(void);
  if((*((unsigned long *)0x2001C000) == 0xDEADBEEF) || pin || !app_ok()) {  //Memory map, datasheet
    *((unsigned long *)0x2001C000) = 0xCAFEFEED;                            //Reset bootloader trigger
    __set_MSP(0x20001000);
    //Point the PC to the System Memory reset vector (+4)
    //AN2606
    //Table 64. Bootloader device-dependent parameters
    SysMemBootJump = (void (*)(void))(*((uint32_t *)0x1FFF0004));
    SysMemBootJump();
    while(1)
      ;
  } else {
    uint32_t stack = ((const uint32_t *)APP_START)[0];
    uint32_t entry = ((const uint32_t *)APP_START)[1];
    asm volatile(
        "msr    msp, %0        \n\t"
        "bx     %1             \n\t"
        :
        : "r"(stack), "r"(entry));
    while(1)
      ;
  }
}
