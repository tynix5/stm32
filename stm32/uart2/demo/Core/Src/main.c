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
#include <stdio.h>
#include "stm32f4xx.h"
#include "uart2.h"

/* printf() uses __io_putchar() to print strings */
/* Override __io_putchar (defined as weak) so that the characters
 * are redirected to UART2 port
 */
int __io_putchar(int ch) {

	uart2_writebyte((uint8_t)ch);			// use uart2 to send character
	return ch;
}

int main(void)
{

	uart2_init(115200);

	while (1)
	{
		printf("Test UART2...\n\r");		// should print this in console shell/realterm
	}

}
