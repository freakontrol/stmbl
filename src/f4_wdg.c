// F4 independent watchdog in hardware-watchdog mode, see inc/f4_wdg.h.

#include "f4_wdg.h"
#include "hal.h"
#include "commands.h"
#include "defines.h"
#include "main.h"
#include <stdio.h>

// LSI / 32 with RLR 1500: 1.5 s at the 32 kHz typical LSI, 1.0 s at the
// fastest (47 kHz) and 2.8 s at the slowest (17 kHz, DS8626). Longer than the
// slowest flash erase the nrt waits for (16 KB sector, 500 ms max, DS8626
// Table 40). The nrt kicks it (main.c Wait()), so a stopped rt alone does not
// reset: "stop" and reloading comps is normal use, and the F3 drops the
// drive when the F4 stops sending.
#define WDG_PR 3  // /32
#define WDG_RLR 1500

void wdg_init(void) {
  wdg_kick();
  // stop the IWDG while a debugger halts the core (RM0090 38.16.4)
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  if(!wdg_is_hw()) {
    return;
  }
  // the ROM DFU leaves the prescaler at its maximum (AN2606 4.8), so set it
  // again. RM0090 21.3.2: any other KR write between 0x5555 and the RLR write
  // locks PR/RLR again, so the sequence runs with interrupts off.
  __disable_irq();
  IWDG->KR  = 0x5555;
  IWDG->PR  = WDG_PR;
  IWDG->RLR = WDG_RLR;
  for(uint32_t t = 0; IWDG->SR && t < 1000000; t++) {
  }
  IWDG->KR = 0xAAAA;
  __enable_irq();
}

// FLASH_OPTCR WDG_SW written with the RM0090 3.7.2 sequence: unlock with
// OPTKEY1/2, write the option value, OPTSTRT, wait for BSY. A reset applies it
// (RM0090 3.9.10 note).
static void wdg_set_mode(uint32_t sw) {
  hal_stop();
  __disable_irq();
  while(FLASH->SR & FLASH_SR_BSY) {
    wdg_kick();
  }
  FLASH->OPTKEYR = 0x08192A3B;
  FLASH->OPTKEYR = 0x4C5D6E7F;
  uint32_t optcr = FLASH->OPTCR & ~(FLASH_OPTCR_OPTSTRT | FLASH_OPTCR_OPTLOCK);
  optcr          = sw ? (optcr | FLASH_OPTCR_WDG_SW) : (optcr & ~FLASH_OPTCR_WDG_SW);
  FLASH->OPTCR   = optcr;
  FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
  while(FLASH->SR & FLASH_SR_BSY) {
    wdg_kick();
  }
  FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
  NVIC_SystemReset();
}

void wdg(char *ptr) {
  if(wdg_is_hw()) {
    printf("watchdog: hardware IWDG (WDG_SW = 0), PR %lu RLR %lu (%u ms at the 32 kHz LSI), kicked by the nrt\n", IWDG->PR, IWDG->RLR, WDG_RLR * 32 * 1000 / 32000);
  } else {
    printf("watchdog: off (WDG_SW = 1); wdg_hw turns it on\n");
  }
}
COMMAND("wdg", wdg, "show the watchdog mode");

void wdg_hw(char *ptr) {
  if(wdg_is_hw()) {
    printf("already in hardware watchdog mode\n");
    return;
  }
  printf("setting option bit WDG_SW = 0 (hardware watchdog), then reset.\n");
  printf("an old app without wdg support reset-loops on this board; BOOT0 DFU still works.\n");
  Wait(20);
  wdg_set_mode(0);
}
COMMAND("wdg_hw", wdg_hw, "persistent: option bit WDG_SW = 0, hardware watchdog from power-up, then reset");

void wdg_sw(char *ptr) {
  if(!wdg_is_hw()) {
    printf("watchdog already off\n");
    return;
  }
  printf("setting option bit WDG_SW = 1 (watchdog off), then reset.\n");
  Wait(20);
  wdg_set_mode(1);
}
COMMAND("wdg_sw", wdg_sw, "persistent: option bit WDG_SW = 1, watchdog off, then reset");
