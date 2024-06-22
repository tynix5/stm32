
#ifndef DMA2_H_
#define DMA2_H_

#include "stm32f4xx.h"

void DMA2_MemToMem_Config();
void DMA2_PeriphToMem_Config(uint8_t channel);
void DMA2_Transfer(uint16_t * dest, uint16_t * src, uint16_t size);

#endif