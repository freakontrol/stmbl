#pragma once
#include "stm32f4xx.h"

// Stop a DMA stream so it can be reconfigured or re-armed. RM0090 10.3.17:
// clearing EN "is not immediately effective", the stream is ready only when
// EN reads back 0, and it re-enables only with all its flags (FEIF, DMEIF,
// TEIF, HTIF, TCIF) cleared. NDTR is writable only while EN is 0 (10.5.6).
static inline void dma_stream_stop(DMA_Stream_TypeDef *stream) {
  stream->CR &= ~DMA_SxCR_EN;
  for(uint32_t i = 0; i < 10000 && (stream->CR & DMA_SxCR_EN); i++) {
  }
  DMA_TypeDef *dma = (uint32_t)stream < (uint32_t)DMA2 ? DMA1 : DMA2;
  uint32_t n       = ((uint32_t)stream - (uint32_t)dma - 0x10) / 0x18;
  static const uint8_t shift[4] = {0, 6, 16, 22};
  uint32_t flags = 0x3Du << shift[n & 3];
  if(n < 4) {
    dma->LIFCR = flags;
  } else {
    dma->HIFCR = flags;
  }
}
