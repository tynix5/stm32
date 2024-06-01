
#include "dma2.h"
#include "stm32f4xx.h"

#define DMA2_EN             (1U << 22)
#define DMA_SCR_EN          (1U << 0)
#define DMA_SCR_MINC        (1U << 10)
#define DMA_SCR_PINC        (1U << 9)
#define DMA_SCR_TCIE        (1U << 4)
#define DMA_SCR_TEIE        (1U << 2)
#define DMA_SFCR_DMDIS      (1U << 2)

void dma2_memtomem_config() {

    // Enable clock access for DMA2 module
    RCC->AHB1ENR |= DMA2_EN;

    // Disable DMA stream
    DMA2_Stream0->CR = 0;

    // Wait until stream is disabled
    while ((DMA2_Stream0->CR & DMA_SCR_EN));

    // Set MSIZE (i.e. memory data size) to half-word (16 bit)
    DMA2_Stream0->CR |= (1U << 13);
    DMA2_Stream0->CR &= ~(1U << 14);

    // Set PSIZE (i.e. peripheral data size) to half-word
    DMA2_Stream0->CR |= (1U << 11);
    DMA2_Stream0->CR &= ~(1U << 12);

    // Set memory increment mode
    DMA2_Stream0->CR |= DMA_SCR_MINC;

    // Enable peripheral address increment
    DMA2_Stream0->CR |= DMA_SCR_PINC;  

    // Enable transfer complete interrupt enable
    DMA2_Stream0->CR |= DMA_SCR_TCIE;  

    // Enable transfer error interrupt enable
    DMA2_Stream0->CR |= DMA_SCR_TEIE;  

    // Select mem-to-mem mode
    DMA2_Stream0->CR &= ~(1U << 6);
    DMA2_Stream0->CR |= (1U << 7);

    // Disable direct mode
    DMA2_Stream0->FCR |= DMA_SFCR_DMDIS;

    // Set FIFO threshold
    DMA2_Stream0->FCR |= (1U << 0);  
    DMA2_Stream0->FCR |= (1U << 1);  

    // Enable DMA interrupt in NVIC
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


void dma2_transfer(uint16_t * dest, uint16_t * src, uint16_t size) {

    // Set peripheral address
    DMA2_Stream0->PAR = src;

    // Set memory address
    DMA2_Stream0->M0AR = dest;

    // Set transfer length (in words)
    DMA2_Stream0->NDTR = size;

    // Enable DMA stream
    DMA2_Stream0->CR |= DMA_SCR_EN;
}