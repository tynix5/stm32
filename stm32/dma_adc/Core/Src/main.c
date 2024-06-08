
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Calculates average value of ADC over 100 readings using DMA
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

#include "main.h"
#include "dma2.h"
#include "uart2.h"
#include <stdio.h>

#define NUM_SAMPLES 100

uint16_t samples[NUM_SAMPLES] = {0};

void ADC1_DMA_Init();
void ADC1_Start();

int main(void)
{

  UART2_Init(115200);

  // Initialize DMA2 on stream 0, channel 0 (ADC1)
  DMA2_PeriphToMem_Config(DMA_CHANNEL_0);

  // Initialize ADC1
  ADC1_DMA_Init();

  // Enable DMA interrupt in NVIC on selected stream
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  // Enable DMA transfer
  DMA2_Transfer(samples, (uint16_t *)&(ADC1->DR), NUM_SAMPLES);

  ADC1_Start();

  while (1)
  {
  }
}

void ADC1_DMA_Init()
{

  // Enable GPIOA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // Set ADC Channel 1 Pin 0 (PA0) as analog input
  GPIOA->MODER |= GPIO_MODE_ANALOG << GPIO_MODER_MODE0_Pos;

  // Enable ADC clock
  RCC->APB2ENR = RCC_APB2ENR_ADC1EN;

  // Configure ADC
  ADC1->CR1 = ADC_CR1_SCAN; // 12 bit precision, no interrupts, analog watchdog disabled, scan regular channels (only PA0 here)

  ADC1->CR2 = ADC_CR2_DDS |  // DMA requests are issued forever (no need to restart DMA)
              ADC_CR2_DMA |  // enable DMA
              ADC_CR2_CONT | // enable ADC for continuous conversion
              ADC_CR2_ADON;  // ADC enabled

  ADC1->SMPR1 = 0;
  ADC1->SMPR2 = ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0; // 480 sample cycles for one conversion on channel 0
                                                                        // anything faster than this does not work (change ADCPRE instead?)

  ADC1->SQR1 = 0; // 1 conversion per sequence
  ADC1->SQR2 = 0;
  ADC1->SQR3 = 0; // first (and only) conversion in sequence is ADC Channel 0

  // ADC1->CCR = ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0; // adc clock = clock/8
}

void ADC1_Start()
{

  ADC1->CR2 |= ADC_CR2_SWSTART; // start conversions
}

void DMA2_Stream0_IRQHandler()
{

  if ((DMA2->LISR) & DMA_LISR_TCIF0)
  { // transfer complete interrupt

    int avg = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
      avg += samples[i];

    avg /= NUM_SAMPLES;

    printf("Average: %d\r\n", avg);

    DMA2->LIFCR |= DMA_LIFCR_CTCIF0; // reset complete flag

  }
  else if ((DMA2->LISR) & DMA_LISR_TEIF0)
  { // error interrupt flag

    printf("Error during transfer\n\r");
    DMA2->LIFCR |= DMA_LIFCR_CTEIF0; // reset error flag
  }
}
