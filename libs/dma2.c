
#include "dma2.h"
#include "stm32f4xx.h"

void DMA2_MemToMem_Config()
{

    // Enable clock access for DMA2 module
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable DMA stream
    DMA2_Stream0->CR = 0;

    // Wait until stream is disabled
    while ((DMA2_Stream0->CR & DMA_SxCR_EN))
    {
    }

    // Configure DMA (channel is irrevelent for mem-to-mem)
    DMA2_Stream0->CR = DMA_SxCR_MSIZE_0 | // memory data size to half-word (16 bit)
                       DMA_SxCR_PSIZE_0 | // peripheral data size to half-word (16 bit)
                       DMA_SxCR_MINC |    // memory increment mode
                       DMA_SxCR_PINC |    // peripheral increment mode
                       DMA_SxCR_TCIE |    // transfer complete interrupt enable
                       DMA_SxCR_TEIE |    // transfer error interrupt enable
                       DMA_SxCR_DIR_1;    // memory to memory type transfer

    // Configure FIFO
    DMA2_Stream0->FCR = DMA_SxFCR_DMDIS |                // disable direct mode
                        DMA_SxFCR_FS_1 | DMA_SxFCR_FS_0; // FIFO status 3/4 <= level < full

    // Enable DMA interrupt in NVIC
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void DMA2_PeriphToMem_Config(uint8_t channel)
{

    // Enable clock access for DMA2 module
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable DMA stream
    DMA2_Stream0->CR = 0;

    // Wait until stream is disabled
    while ((DMA2_Stream0->CR & DMA_SxCR_EN))
    {
    }

    // Configure DMA as peripheral to memory type transfer
    DMA2_Stream0->CR = ((channel & 0x07) << DMA_SxCR_CHSEL_Pos) | // channel selection
                       DMA_SxCR_MSIZE_0 |                         // memory data size to half-word (16 bit)
                       DMA_SxCR_PSIZE_0 |                         // peripheral data size to half-word (16 bit)
                       DMA_SxCR_MINC |                            // memory increment mode
                       DMA_SxCR_CIRC |                            // circular mode --> transaction size must be a multiple of bursts * (msize/psize) = 4
                       DMA_SxCR_TCIE |                            // transfer complete interrupt enable
                       DMA_SxCR_TEIE;                             // transfer error interrupt enable

    // Configure FIFO
    DMA2_Stream0->FCR = DMA_SxFCR_DMDIS |                // disable direct mode
                        DMA_SxFCR_FS_1 | DMA_SxFCR_FS_0; // FIFO status 3/4 <= level < full

    // Enable NVIC in source since stream is unknown
}

void DMA2_Transfer(uint16_t *dest, uint16_t *src, uint16_t size)
{

    // Set peripheral address (source is loaded on peripheral bus in mem-to-mem)
    DMA2_Stream0->PAR = (uint32_t)src;

    // Set memory address (dest is written from memory bus in mem-to-mem)
    DMA2_Stream0->M0AR = (uint32_t)dest;

    // Set transfer length (in words)
    DMA2_Stream0->NDTR = size;

    // Enable DMA stream
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}