
#ifndef DMA_H_
#define DMA_H_


void dma2_memtomem_config();
void dma2_transfer(uint16_t * dest, uint16_t * src, uint16_t size);


#endif