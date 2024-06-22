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

#include <stdio.h>
#include "stm32f4xx.h"

#define GPIOAEN		(1 << 0)


int main(void)
{
  
  RCC->AHB1ENR |= GPIOAEN;

  GPIOA->MODER |= (1 << 10);
  GPIOA->MODER &= ~(1 << 11);

  while (1)
  {
	  GPIOA->ODR ^= (1 << 5);

	  for (volatile int i = 0; i < 50000; i++);
  }

}

