
#ifndef DMA2_H_
#define DMA2_H_

#include "stm32f4xx.h"

void dma2_memtomem_config();
void dma2_transfer(uint16_t * dest, uint16_t * src, uint16_t size);

#endif