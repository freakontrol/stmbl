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

#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "commands.h"
#include "stm32f4xx_conf.h"

// Flash sector erase and byte program on registers (RM0090 3.6; LL has no
// flash API on F4). 2.7-3.6 V supply, so erase runs at PSIZE x32. The CPU
// stalls on flash reads while an operation runs, as it did with HAL.
#define FLASH_TIMEOUT_MS 50000U  // as HAL's FLASH_TIMEOUT_VALUE
#define FLASH_ERR_FLAGS (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

extern volatile uint64_t systime;

// wait for BSY to drop, clear EOP and any error flags, return the errors
static uint32_t flash_wait(void) {
  uint64_t start = systime;
  while(FLASH->SR & FLASH_SR_BSY) {
    if(systime - start > FLASH_TIMEOUT_MS) {
      return FLASH_SR_BSY;
    }
  }
  uint32_t err = FLASH->SR & FLASH_ERR_FLAGS;
  FLASH->SR    = FLASH_SR_EOP | err;  // rc_w1
  return err;
}

static void flash_unlock(void) {
  if(FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123U;
    FLASH->KEYR = 0xCDEF89ABU;
  }
  FLASH->SR = FLASH_SR_EOP | FLASH_ERR_FLAGS;  // stale flags would fail the first operation
}

static void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

// the ART caches may hold the old contents of an erased sector
static void flash_flush_caches(void) {
  if(READ_BIT(FLASH->ACR, FLASH_ACR_ICEN)) {  // LL has no getter for the caches
    LL_FLASH_DisableInstCache();
    LL_FLASH_EnableInstCacheReset();
    LL_FLASH_DisableInstCacheReset();
    LL_FLASH_EnableInstCache();
  }
  if(READ_BIT(FLASH->ACR, FLASH_ACR_DCEN)) {
    LL_FLASH_DisableDataCache();
    LL_FLASH_EnableDataCacheReset();
    LL_FLASH_DisableDataCacheReset();
    LL_FLASH_EnableDataCache();
  }
}

static uint32_t flash_erase_sector(uint32_t sector) {
  uint32_t err = flash_wait();
  if(!err) {
    MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE | FLASH_CR_SNB, FLASH_CR_PSIZE_1 | (sector << FLASH_CR_SNB_Pos));
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    err = flash_wait();
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  }
  flash_flush_caches();
  return err;
}

static uint32_t flash_program_byte(uint32_t addr, uint8_t data) {
  uint32_t err = flash_wait();
  if(!err) {
    MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE, 0);  // x8
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint8_t *)addr = data;
    err = flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
  }
  return err;
}

char config[15 * 1024];
const char *config_ro = (char *)0x08008000;

void confcrc(char *ptr) {
  uint32_t len = strnlen(config, sizeof(config) - 1);
  CRC->CR = CRC_CR_RESET;
  uint32_t crc = crc_calc_block((uint32_t *)config, len / 4);
  for(int i = 0; i < len; i++) {
    printf("%x ", config[i]);
  }
  printf("\n");
  printf("size: %lu words: %lu crc:%lx\n", len, len / 4, crc);
}
COMMAND("confcrc", confcrc, "foo");

void flashloadconf(char *ptr) {
  strncpy(config, config_ro, sizeof(config));
}
COMMAND("flashloadconf", flashloadconf, "load config from flash");

void flashsaveconf(char *ptr) {
  printf("erasing flash page...\n");
  flash_unlock();
  if(flash_erase_sector(2)) {  // 0x08008000, 16 KB: config_ro
    printf("error!\n");
    flash_lock();
    return;
  }
  printf("saving conf\n");
  int i   = 0;
  int ret = 0;
  do {
    ret = flash_program_byte((uint32_t)(config_ro + i), config[i]) != 0;
    if(ret) {
      printf("error writing %i\n", ret);
      break;
    }
  } while(config[i++] != 0);
  printf("OK %i bytes written\n", i);
  flash_lock();
}
COMMAND("flashsaveconf", flashsaveconf, "save config to flash");

void loadconf(char *ptr) {
  hal_parse(config);
}
COMMAND("loadconf", loadconf, "parse config");

void showconf(char *ptr) {
  printf("%s", config_ro);
}
COMMAND("showconf", showconf, "show config");

void appendconf(char *ptr) {
  printf("adding %s\n", ptr);
  strncat(config, ptr, sizeof(config) - 1);
  strncat(config, "\n", sizeof(config) - 1);
}
COMMAND("appendconf", appendconf, "append string to config");

void deleteconf(char *ptr) {
  config[0] = '\0';
}
COMMAND("deleteconf", deleteconf, "delete config");

void hardboot(char *ptr) {
  printf("erasing flash page...\n");
  flash_unlock();
  if(flash_erase_sector(4)) {  // 0x08010000, 64 KB: the app's first sector
    printf("error!\n");
    flash_lock();
    return;
  }
  printf("OK, call bootloader\n");
  flash_lock();
  NVIC_SystemReset();
}
COMMAND("hardboot", hardboot, "destroy firmware to force bootloader");