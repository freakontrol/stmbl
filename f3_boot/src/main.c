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
#include "stm32f3xx.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_rtc.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_crc.h"
#include "version.h"
#include "common.h"
#include "f3hw.h"

uint32_t systick_freq;
volatile uint64_t systime = 0;

volatile packet_bootloader_t rx_buf;
volatile packet_bootloader_t tx_buf;


void SystemClock_Config(void);
void Error_Handler(void);

void SysTick_Handler(void) {
  systime++;
}

// Busy wait of at least ms full SysTick periods (the partial current one is not counted)
static void delay_ms(uint32_t ms) {
  uint64_t start = systime;
  while(systime - start < (uint64_t)ms + 1) {
  }
}

// CRC-32 over words from the reset value, CRC unit in its reset configuration
static uint32_t crc_calc(const uint32_t *buf, uint32_t len) {
  LL_CRC_ResetCRCCalculationUnit(CRC);
  for(uint32_t i = 0; i < len; i++) {
    LL_CRC_FeedData32(CRC, buf[i]);
  }
  return LL_CRC_ReadData32(CRC);
}

// Flash program/erase on registers (LL has no flash API on F3): unlock, PG or
// PER, BSY polled, EOP and error flags cleared; the sequences of
// HAL_FLASH_Program()/HAL_FLASHEx_Erase(), kept as validated
#define FLASH_TIMEOUT_MS 50000U
#define FLASH_PAGE_SIZE 0x800U  // 2 KB pages on the F303xC

static int flash_wait(void) {
  uint64_t start = systime;
  while(FLASH->SR & FLASH_SR_BSY) {
    if(systime - start > FLASH_TIMEOUT_MS) {
      return -1;
    }
  }
  if(FLASH->SR & FLASH_SR_EOP) {
    FLASH->SR = FLASH_SR_EOP;
  }
  if(FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGERR)) {
    FLASH->SR = FLASH_SR_WRPERR | FLASH_SR_PGERR;
    return -1;
  }
  return 0;
}

static void flash_unlock(void) {
  if(FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

static void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static int flash_program_word(uint32_t addr, uint32_t data) {
  if(flash_wait()) {
    return -1;
  }
  for(uint32_t i = 0; i < 2; i++) {
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t *)(addr + 2U * i) = (uint16_t)(data >> (16U * i));
    int ret = flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    if(ret) {
      return -1;
    }
  }
  return 0;
}

static int flash_erase_pages(uint32_t addr, uint32_t end) {
  if(flash_wait()) {
    return -1;
  }
  for(; addr < end; addr += FLASH_PAGE_SIZE) {
    // Skip pages that are already blank. The erase runs in the TIM8 interrupt
    // and answers once; when that answer is lost the f4 sends PAGEERASE
    // again, and a full 56 page erase each time (1-2 s) ran it out of time.
    // A repeat now answers within a few ms.
    const uint32_t *p = (const uint32_t *)addr;
    uint32_t blank    = 0xFFFFFFFFu;
    for(uint32_t i = 0; i < FLASH_PAGE_SIZE / 4; i++) {
      blank &= p[i];
    }
    if(blank == 0xFFFFFFFFu) {
      continue;
    }
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    int ret = flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    if(ret) {
      return -1;
    }
  }
  return 0;
}

#define APP_START 0x08004000
#define APP_END 0x08020000
#define APP_RANGE_VALID(a, s) (!(((a) | (s)) & 3) && (a) >= APP_START && ((a) + (s)) <= APP_END)
#define VERSION_INFO_OFFSET 0x188
static volatile const version_info_t *app_info = (void *)(APP_START + VERSION_INFO_OFFSET);

static int app_ok(void) {
  if(!APP_RANGE_VALID(APP_START, app_info->image_size)) {
    return 0;
  }
  uint32_t crc = crc_calc((uint32_t *)APP_START, app_info->image_size / 4);

  if(crc != 0) {
    return 0;
  }
  return 1;
}

void TIM8_UP_IRQHandler() {
  static uint32_t last_dma_count = 0;

  uint32_t dma_count = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
  // if(USART3->ISR & USART_ISR_RTOF) {                                    // idle line
  //   USART3->ICR |= USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF;  // timeout clear flag
  if(dma_count == last_dma_count) {  // framing
    last_dma_count = 0;

    // start rx DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_bootloader_t));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
  } else {
    last_dma_count = dma_count;
  }

  if(dma_count == 0) {
    if(rx_buf.header.slave_addr == 255 && rx_buf.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4 && rx_buf.header.crc == crc_calc((uint32_t *)&(rx_buf.header.slave_addr), sizeof(packet_bootloader_t) / 4 - 1)) {
      //do stuff
      //tx_buf.state = do_stuff();

      switch(rx_buf.header.flags.cmd) {
        case NO_CMD:
          break;
        case WRITE_CONF:
          break;
        case READ_CONF:
          break;
        case DO_RESET:
          flash_lock();
          NVIC_SystemReset();
          break;
        case BOOTLOADER:
          break;
      }

      int status = 0;
      switch(rx_buf.cmd) {
        case BOOTLOADER_OPCODE_NOP:
          break;
        case BOOTLOADER_OPCODE_READ:
          tx_buf.value = *(uint32_t *)rx_buf.addr;
          tx_buf.addr  = rx_buf.addr;
          break;

        case BOOTLOADER_OPCODE_WRITE:
          if(*(uint32_t *)rx_buf.addr != rx_buf.value) {
            status = flash_program_word(rx_buf.addr, rx_buf.value);
          }
          if(*(uint32_t *)rx_buf.addr != rx_buf.value) {
            status = -1;
          }
          tx_buf.value = *(uint32_t *)rx_buf.addr;
          tx_buf.addr  = rx_buf.addr;
          break;

        case BOOTLOADER_OPCODE_PAGEERASE:
          flash_unlock();
          status = flash_erase_pages(APP_START, APP_END);
          break;

        case BOOTLOADER_OPCODE_CRCCHECK:
          status = app_ok() ? 0 : -1;
          break;
      }

      if(status != 0) {
        tx_buf.state = BOOTLOADER_STATE_NAK;
      } else {
        tx_buf.state = BOOTLOADER_STATE_OK;
      }

      tx_buf.cmd                  = rx_buf.cmd;
      tx_buf.header.flags.counter = rx_buf.header.flags.counter;
      tx_buf.header.slave_addr    = 255;
      tx_buf.header.len           = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      tx_buf.header.conf_addr     = 0;
      tx_buf.header.config.u32    = 0;
      tx_buf.header.flags.cmd     = NO_CMD;
      tx_buf.header.crc           = crc_calc((uint32_t *)&(tx_buf.header.slave_addr), sizeof(packet_bootloader_t) / 4 - 1);

      rx_buf.header.crc = 0;

      // start tx DMA
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(packet_bootloader_t));
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    }
  }

  LL_TIM_ClearFlag_UPDATE(TIM8);
}

void uart_init() {
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  // 8N1, 8x oversampling, clocked from SYSCLK; DMA requests and overrun
  // disable are set before the init, which keeps them (the order the HAL
  // init used and the link was validated with)
  LL_USART_EnableDMAReq_TX(USART3);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_DisableOverrunDetect(USART3);
  LL_USART_Disable(USART3);
  LL_USART_Init(USART3, &(LL_USART_InitTypeDef){
                            .BaudRate            = DATABAUD,
                            .DataWidth           = LL_USART_DATAWIDTH_8B,
                            .StopBits            = LL_USART_STOPBITS_1,
                            .Parity              = LL_USART_PARITY_NONE,
                            .TransferDirection   = LL_USART_DIRECTION_TX_RX,
                            .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
                            .OverSampling        = LL_USART_OVERSAMPLING_8,
                        });
  LL_USART_DisableOneBitSamp(USART3);
  LL_USART_ConfigAsyncMode(USART3);  // LINEN, CLKEN, SCEN, IREN, HDSEL off
  LL_USART_Enable(USART3);
  while(!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {
  }

  /**USART3 GPIO Configuration    
   PB10     ------> USART3_TX
   PB11     ------> USART3_RX 
   */
  LL_GPIO_Init(GPIOB, &(LL_GPIO_InitTypeDef){
                          .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
                          .Mode       = LL_GPIO_MODE_ALTERNATE,
                          .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
                          .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                          .Pull       = LL_GPIO_PULL_UP,
                          .Alternate  = LL_GPIO_AF_7,
                      });

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  //TX DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&tx_buf, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, sizeof(packet_bootloader_t));
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_LOW);
  LL_DMA_ClearFlag_GI2(DMA1);

  //RX DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)&rx_buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_bootloader_t));
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_MODE_NORMAL | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                            LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE | LL_DMA_PRIORITY_LOW);
  LL_DMA_ClearFlag_GI3(DMA1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

  LL_USART_SetRxTimeout(USART3, 16);  // 16 bits timeout
  LL_USART_EnableRxTimeout(USART3);
  LL_USART_ClearFlag_RTO(USART3);
}

/* RTC init: 24h, prescalers 127/255, no output, only if the calendar was never
   set up (INITS clear); BKP0R carries the bootloader request across resets */
static void rtc_init(void) {
  if(LL_RTC_IsActiveFlag_INITS(RTC)) {
    return;
  }
  LL_RTC_DisableWriteProtection(RTC);
  if(!LL_RTC_IsActiveFlag_INIT(RTC)) {
    LL_RTC_EnableInitMode(RTC);
    uint64_t start = systime;
    while(!LL_RTC_IsActiveFlag_INIT(RTC)) {
      if(systime - start > 1000) {
        LL_RTC_EnableWriteProtection(RTC);
        Error_Handler();
      }
    }
  }
  LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
  LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
  LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
  LL_RTC_SetSynchPrescaler(RTC, 255);
  LL_RTC_SetAsynchPrescaler(RTC, 127);
  LL_RTC_DisableInitMode(RTC);
  if(!LL_RTC_IsShadowRegBypassEnabled(RTC)) {
    LL_RTC_ClearFlag_RS(RTC);
    uint64_t start = systime;
    while(!LL_RTC_IsActiveFlag_RS(RTC)) {
      if(systime - start > 1000) {
        LL_RTC_EnableWriteProtection(RTC);
        Error_Handler();
      }
    }
  }
  LL_RTC_SetAlarmOutputType(RTC, LL_RTC_ALARM_OUTPUTTYPE_OPENDRAIN);
  LL_RTC_EnableWriteProtection(RTC);
}

// TIM8 only times the rx framing here: 144 MHz / (2 * PWM_RES / 2), no outputs,
// no break, update interrupt at priority 15
static void tim8_init(void) {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
  NVIC_SetPriority(TIM8_UP_IRQn, 15);
  NVIC_EnableIRQ(TIM8_UP_IRQn);

  LL_TIM_SetCounterMode(TIM8, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);
  LL_TIM_SetClockDivision(TIM8, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_DisableARRPreload(TIM8);
  LL_TIM_SetAutoReload(TIM8, PWM_RES / 2);
  LL_TIM_SetPrescaler(TIM8, 0);
#ifdef PWM_INVERT
  LL_TIM_SetRepetitionCounter(TIM8, 1);
#else
  LL_TIM_SetRepetitionCounter(TIM8, 0);
#endif
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_COUNTER);  // load PSC/RCR without setting UIF
  LL_TIM_GenerateEvent_UPDATE(TIM8);
  LL_TIM_SetUpdateSource(TIM8, LL_TIM_UPDATESOURCE_REGULAR);

  LL_TIM_SetClockSource(TIM8, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetSlaveMode(TIM8, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_SetTriggerInput(TIM8, LL_TIM_TS_ITR0);
  LL_TIM_ConfigETR(TIM8, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_DisableMasterSlaveMode(TIM8);
  LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_RESET);
  LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_UPDATE);
  // dead time, break polarities high, breaks, outputs and off states disabled
  LL_TIM_OC_SetDeadTime(TIM8, PWM_DEADTIME);
  LL_TIM_ConfigBRK(TIM8, LL_TIM_BREAK_POLARITY_HIGH, LL_TIM_BREAK_FILTER_FDIV1);
  LL_TIM_ConfigBRK2(TIM8, LL_TIM_BREAK2_POLARITY_HIGH, LL_TIM_BREAK2_FILTER_FDIV1);
}

int main(void) {
  extern void *g_pfnVectors;
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  SystemClock_Config();
  systick_freq = SystemCoreClock;

  /* Initialize all configured peripherals */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMA2);
  LL_RCC_EnableRTC();

  /*Configure GPIO pin Output Level */
  LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
  LL_GPIO_Init(LED_PORT, &(LL_GPIO_InitTypeDef){.Pin = LED_PIN, .Mode = LL_GPIO_MODE_OUTPUT, .Speed = LL_GPIO_SPEED_FREQ_LOW, .OutputType = LL_GPIO_OUTPUT_PUSHPULL, .Pull = LL_GPIO_PULL_NO});

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);  // reset config: CRC-32, init 0xFFFFFFFF, no reversal

  rtc_init();

  if(app_ok() && LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) == 0x00000000) {
    /* Jump to user application */
    void (*JumpToApplication)(void);
    uint32_t JumpAddress = *(__IO uint32_t *)(APP_START + 4);
    JumpToApplication    = (void *)JumpAddress;

    // stop the 1 kHz tick: until the app sets VTOR it would run this
    // bootloader's SysTick_Handler on the app's RAM
    SysTick->CTRL = 0;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;

    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t *)APP_START);
    JumpToApplication();
    while(1) {
    }
  } else {
    uart_init();

    tx_buf.header.slave_addr = 255;
    tx_buf.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
    tx_buf.header.conf_addr  = 0;
    tx_buf.header.config.u32 = 0;
    tx_buf.header.flags.cmd  = NO_CMD;

    // start rx DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(packet_bootloader_t));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

    tim8_init();
    LL_TIM_EnableIT_UPDATE(TIM8);
    LL_TIM_EnableCounter(TIM8);
  }
  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0x00000000);

  while(1) {
    LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
    delay_ms(50);
    LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
    delay_ms(50);
  }
}

// HSE 8 MHz * 9 = 72 MHz, APB1 36 MHz, APB2 72 MHz, TIM8 from the PLL,
// USART3 from SYSCLK, RTC from LSI, SysTick 1 kHz at priority 0.
void SystemClock_Config(void) {
  LL_FLASH_EnablePrefetch();
  NVIC_SetPriorityGrouping(3);  // 4 bits preemption, no subpriority
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  NVIC_SetPriority(MemoryManagement_IRQn, 0);
  NVIC_SetPriority(BusFault_IRQn, 0);
  NVIC_SetPriority(UsageFault_IRQn, 0);
  NVIC_SetPriority(SVCall_IRQn, 0);
  NVIC_SetPriority(DebugMonitor_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0);

  LL_RCC_HSE_Enable();
  while(!LL_RCC_HSE_IsReady()) {
  }
  LL_RCC_LSI_Enable();
  while(!LL_RCC_LSI_IsReady()) {
  }
  LL_RCC_PLL_Disable();
  while(LL_RCC_PLL_IsReady()) {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();
  while(!LL_RCC_PLL_IsReady()) {
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_16);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  SystemCoreClockUpdate();

  // RTC clock from LSI; the backup domain stays writable for RTC BKP0R
  uint32_t pwr_was_off = !LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  while(!LL_PWR_IsEnabledBkUpAccess()) {
  }
  uint32_t rtcsel = LL_RCC_GetRTCClockSource();
  if(rtcsel != LL_RCC_RTC_CLKSOURCE_NONE && rtcsel != LL_RCC_RTC_CLKSOURCE_LSI) {
    uint32_t bdcr = LL_RCC_ReadReg(BDCR) & ~RCC_BDCR_RTCSEL;
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_WriteReg(BDCR, bdcr);
  }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  if(pwr_was_off) {
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM8_CLKSOURCE_PLL);

  SysTick_Config(SystemCoreClock / 1000);  // HCLK source
  NVIC_SetPriority(SysTick_IRQn, 0);
}

//Delay implementation for hal_term.c
void Wait(uint32_t ms) {
  delay_ms(ms);
}

void Error_Handler(void) {
  while(1) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
  }
}
