#pragma once

// F4 clock tree on LL, shared by the app and the bootloader (f4_clock.c)
void clock_init(void);
extern uint32_t hse_failed;

// Reset cause: RCC_CSR's reset flags, saved by SystemInit before
// LL_RCC_DeInit clears them (RMVF). SystemInit runs before .data/.bss are
// set up and the F4 bootloader runs it first, so the flags live in SRAM2 next
// to the bootloader's magic word (0x2001C000), outside the app's RAM region,
// with a check word: the app's SystemInit keeps a slot the bootloader filled.
#define RESET_FLAGS_SLOT ((volatile uint32_t *)0x2001C004)
#define RESET_FLAGS_CHECK ((volatile uint32_t *)0x2001C008)
uint32_t reset_flags_take(void);  // the saved flags (0 if none), then forgets them

