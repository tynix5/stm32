/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma2.h"
#include "stm32f4xx.h"
#include "uart2.h"
#include <stdbool.h>
#include <stdio.h>

#define TRANSFER_SIZE       10

uint16_t srcData[TRANSFER_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
uint16_t destData[TRANSFER_SIZE] = {0};

volatile bool transferCmplt = false;

int main() {

  uart2_init(115200);

  // configure DMA2
  dma2_memtomem_config();

  printf("\n\rDestination contents before transfer: ");
  for (int i = 0; i < TRANSFER_SIZE; i++)
    printf("%d ", destData[i]);

  // initiate transfer mem to mem
  dma2_transfer((uint32_t) destData, (uint32_t) srcData, TRANSFER_SIZE);
 
  // wait until transaction complete
  while (!transferCmplt) {}


  while (true) {

    // print out contents of destination
    printf("\n\rDestination contents after transfer: ");
    for (int i = 0; i < TRANSFER_SIZE; i++)
      printf("%d ", destData[i]);

  }

}


void DMA2_Stream0_IRQHandler() {

  // check if transfer complete interrupt occurred
  if ((DMA2->LISR) & DMA_LISR_TCIF0) 
  {
    transferCmplt = true;
    // clear transfer complete flag
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

  }
  // else check if error interrupt occurred
  else if ((DMA2->LISR) & DMA_LISR_TEIF0) {

    // print error to terminal
    transferCmplt = false;
    // clear error interrupt flag
    DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
    printf("\nError during DMA2 transfer");
  }
}