#pragma once
#include "stm32f4xx.h"

// F4 independent watchdog, used only in hardware-watchdog mode (option bit
// WDG_SW = 0, set once per board with the wdg_hw command). Then the IWDG runs
// from power-up and the ROM DFU bootloader keeps it fed (AN2606 Table 73).
// It is never started from software: a software-started IWDG is not fed by
// the ROM DFU, which then resets mid-flash (bench, 2026-09-29).

// RM0090 3.9.10 FLASH_OPTCR: WDG_SW = 0 selects the hardware watchdog
static inline int wdg_is_hw(void) {
  return !(FLASH->OPTCR & FLASH_OPTCR_WDG_SW);
}

// reload; has no effect while the IWDG is not running
static inline void wdg_kick(void) {
  IWDG->KR = 0xAAAA;
}

void wdg_init(void);
