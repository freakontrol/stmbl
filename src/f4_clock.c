// F4 clock setup on LL, shared by the app and the bootloader. SystemInit()
// only resets the clock tree (the app is entered from the bootloader with the
// PLL already running), clock_init() brings it to 168 MHz.

#include "stm32f4xx_conf.h"
#include "f4_clock.h"

// HCLK, as LL and HAL expect it (ll_utils, HAL_InitTick): HSI after reset
uint32_t SystemCoreClock = HSI_VALUE;
// prescaler code to shift, for LL_RCC_GetSystemClocksFreq()
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t hse_failed = 0;  // set by clock_init() if the HSE never became ready

#define PLL_M 4    // HSE 8 MHz / 4 = 2 MHz into the PLL (RM0090 6.3.2 recommends 2 MHz for jitter)
#define PLL_N 168  // 336 MHz VCO
#define PLL_P 2    // 168 MHz SYSCLK
#define PLL_Q 7    // 48 MHz USB

// Called from the startup code before main(): FPU on, clocks to their reset
// state (HSI, no PLL, no HSE) so clock_init() always starts from the same
// point, whether after a reset or a jump from the bootloader.
void SystemInit(void) {
  IWDG->KR = 0xAAAA;  // hardware watchdog (WDG_SW = 0): a full period for the start-up, no effect otherwise
  // reset flags (bits 31:25) are there only right after a reset; after the
  // bootloader they are already cleared, so keep what it saved
  uint32_t flags = RCC->CSR & 0xFE000000;
  if(flags) {
    *RESET_FLAGS_SLOT  = flags;
    *RESET_FLAGS_CHECK = ~flags;
  }
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  // CP10, CP11 full access
  // the bootloader enters the app with SYSCLK on the PLL. LL_RCC_DeInit
  // writes SW = HSI and clears PLLON back to back, and PLLON can't be cleared
  // until the switch is done, so finish the switch first (HAL_RCC_DeInit order)
  LL_RCC_HSI_Enable();
  while(!LL_RCC_HSI_IsReady()) {
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
  }
  LL_RCC_DeInit();
}

// HSE 8 MHz / 4 * 168 / 2 = 168 MHz SYSCLK/HCLK, APB1 42 MHz, APB2 84 MHz,
// 48 MHz for USB from PLLQ, flash at 5 wait states with prefetch and both
// caches, regulator scale 1. If the HSE does not start the core stays on
// the 16 MHz HSI and hse_failed is set. The app then does not get past the
// USB setup: the OTG core has no 48 MHz clock and USB_CoreReset spins until
// its HAL timeout (about a minute). With the hardware watchdog (WDG_SW = 0)
// the IWDG resets after 1.5 s, so the board retries until the crystal
// starts (bench, 21 Oct: forced HSE timeout over SWD, reset cause IWDG).
// Either way a dead crystal cannot be reported over USB.
void clock_init(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSE_Enable();
  for(uint32_t t = 0; !LL_RCC_HSE_IsReady(); t++) {
    if(t >= HSE_STARTUP_TIMEOUT) {
      hse_failed = 1;  // main() leaves the hal stopped, "about" says why
      return;
    }
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, PLL_N, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, PLL_N, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();
  while(!LL_RCC_PLL_IsReady()) {
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {  // RM0090 3.5.1: in effect before the switch
  }
  LL_FLASH_EnablePrefetch();
  LL_FLASH_EnableInstCache();
  LL_FLASH_EnableDataCache();

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  SystemCoreClock = HSE_VALUE / PLL_M * PLL_N / PLL_P;

  // RM0090 6.2.7: on an HSE failure the CSS switches to HSI and raises an NMI
  // (NMI_Handler) instead of leaving the PLL without a reference
  LL_RCC_HSE_EnableCSS();
}

uint32_t reset_flags_take(void) {
  uint32_t flags = *RESET_FLAGS_SLOT;
  uint32_t ok    = *RESET_FLAGS_CHECK == ~flags;
  *RESET_FLAGS_CHECK = 0;  // a later jump without a reset (DFU leave) reads 0
  return ok ? flags : 0;
}

