#pragma once

// Small helpers for things STM32CubeF4 LL does not cover.

#include "stm32f4xx.h"

// CRC over 32-bit words, without resetting the CRC unit first
static inline uint32_t crc_calc_block(const uint32_t *buf, uint32_t len) {
  for(uint32_t i = 0; i < len; i++) {
    CRC->DR = buf[i];
  }
  return CRC->DR;
}
