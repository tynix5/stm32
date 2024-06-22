/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : PWM output on PA1
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
#include "stm32f4xx.h"

void TIM2_PWM_Init();
void TIM2_PWM_Start();
void TIM2_PWM_SetDuty(uint8_t duty);

int main(void)
{

  TIM2_PWM_Init();
  TIM2_PWM_Start();
  while (1)
  {

    // increase brightness
    for (int i = 0; i < 100; i++)
    {

      TIM2_PWM_SetDuty(i);
      for (volatile int j = 0; j < 25000; j++)
      {
      }
    }

    // decrease brightness
    for (int i = 99; i > 0; i--)
    {

      TIM2_PWM_SetDuty(i);
      for (volatile int j = 0; j < 25000; j++)
      {
      }
    }
  }
}

void TIM2_PWM_Init()
{

  // enable GPIOA clock
  RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN;

  // select alternate function on PA1
  GPIOA->MODER |= GPIO_MODER_MODE1_1;

  // select alternate function TIM2_CH2 (AF1)
  GPIOA->AFR[0] = GPIO_AFRL_AFRL1_0;

  // enable TIM2 clock
  RCC->APB1ENR = RCC_APB1ENR_TIM2EN;

  // edge-aligned, count up, no clock division, counter disabled
  TIM2->CR1 = 0;
  TIM2->CR2 = 0;

  TIM2->PSC = 10 - 1; // freq = 16 MHz/10 = 1.6 MHz

  TIM2->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | // PWM mode 1 (active when TIM2_CNT < TIM2_CCR2)
                TIM_CCMR1_OC2PE;                      // output compare 2 preload enable

  TIM2->ARR = 100 - 1; // set frequency
  TIM2->CCR2 = 25;     // 25% duty cycle (ARR / 4)

  TIM2->CCER = TIM_CCER_CC2E; // OC2 signal is output on PA1, active high

  TIM2->CR1 = TIM_CR1_ARPE; // auto preload enable
  TIM2->EGR = TIM_EGR_UG;   // re-initialize counter and an update registers
}

void TIM2_PWM_Start()
{

  TIM2->CNT = 0;            // reset counter
  TIM2->CR1 |= TIM_CR1_CEN; // enable counter
}

void TIM2_PWM_SetDuty(uint8_t duty)
{

  if (duty > TIM2->ARR)
    return;

  TIM2->CCR2 = duty; // set duty cycle
}
