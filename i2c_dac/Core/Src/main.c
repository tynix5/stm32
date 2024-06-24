/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : I2C demo - 1Hz sine wave with MCP4725 DAC
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
#include <stdint.h>
#include <math.h>

// I2C Address
#define DAC_WRITE_ADDR 0b1100000
#define DAC_READ_ADDR 0b1100000

// MCP4725 Register Addresses
#define DAC_MODE_FAST 0b000
#define DAC_MODE_WRITE_REG 0b010
#define DAC_MODE_WRITE_REG_EEPROM 0b011

#define DAC_RESOLUTION 4096 // 12 bits

/////////////////////////////////////////////////////
////////////////// I2C Functions ////////////////////
/////////////////////////////////////////////////////
void I2C1_Init();
static inline void I2C1_Start();
static inline void I2C1_WaitStartGen();
static inline void I2C1_Stop();
static inline void I2C1_Ack();
static inline void I2C1_Nack();
uint32_t I2C1_WaitAddrGen();
static inline void I2C1_WaitTXEmpty();
static inline void I2C1_WaitByteTransfer();
void DAC_FastWrite(uint16_t data);
/////////////////////////////////////////////////////
//////////////// End I2C Functions //////////////////
/////////////////////////////////////////////////////

void TIM2_Init();
void TIM2_Start();

uint16_t convertSinToVoltage(double temp);

volatile double angle = 0;
volatile double step = 2 * M_PI / DAC_RESOLUTION; // radians per step

int main()
{

  I2C1_Init();
  TIM2_Init();

  TIM2_Start();

  while (1)
  {

    // atomic sine operation
    NVIC_DisableIRQ(TIM2_IRQn);
    double temp = sin(angle);
    NVIC_EnableIRQ(TIM2_IRQn);

    uint16_t dac = convertSinToVoltage(temp);

    DAC_FastWrite(dac);
  }
}

void I2C1_Init()
{

  // Initialize SCL and SDA lines
  RCC->AHB1ENR = RCC_AHB1ENR_GPIOBEN; // enable GPIOB clock

  GPIOB->MODER |= GPIO_MODER_MODE8_1 | // alternate function mode on PB8
                  GPIO_MODER_MODE9_1;  // alternate function mode on PB9

  GPIOB->AFR[1] = GPIO_AFRH_AFRH0_2 | // select I2C1_SCL as AF for PB8
                  GPIO_AFRH_AFRH1_2;  // select I2C1_SDA as AF for PB9

  GPIOB->OTYPER = GPIO_OTYPER_OT8 | // output open drain on SCL
                  GPIO_OTYPER_OT9;  // output open drain on SDA

  // Initialize I2C in standard mode (<100kHz)
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable I2C clock

  I2C1->CR1 = 0;  // default configuration (I2C disabled)
  I2C1->CR2 = 16; // 16 MHz APB clock frequency (CPU freq)

  // 50 kHz freq on SCL (p. 503)
  I2C1->CCR = 160; // Sm mode (standard)

  I2C1->CR1 = I2C_CR1_PE; // enable peripheral
}

static inline void I2C1_Start()
{
  // generate start condition, master mode
  I2C1->CR1 |= I2C_CR1_START;
}

static inline void I2C1_WaitStartGen()
{
  // wait for start generation
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;

  // start status bit is cleared by writing to DR
}

static inline void I2C1_Stop()
{
  // generate stop condition
  I2C1->CR1 |= I2C_CR1_STOP;
}

static inline void I2C1_Ack()
{
  // generate acknowledge
  I2C1->CR1 |= I2C_CR1_ACK;
}

static inline void I2C1_Nack()
{
  // generate no acknowledge
  I2C1->CR1 &= ~I2C_CR1_ACK;
}

uint32_t I2C1_WaitAddrGen()
{
  volatile unsigned int timeout = 0;
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) // wait until address is sent
  {

    // I2C bus will randomly release lines with no error status bits set
    // Restart communications on timeout
    timeout++;
    if (timeout > 5000)
    {
      timeout = 0;
      return -1;
    }
  }
  return I2C1->SR2; // read SR2 to clear ADDR status bit
}

static inline void I2C1_WaitTXEmpty()
{
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ; // wait until data register empty
}

void I2C1_WaitByteTransfer()
{
  // wait until data register empty and byte transfer complete
  while (!(I2C1->SR1 & I2C_SR1_TXE) && !(I2C1->SR1 & I2C_SR1_BTF));
}

void DAC_FastWrite(uint16_t data)
{

  uint8_t addr = (uint8_t)(DAC_WRITE_ADDR << 1); // 7 bit address + write (LSB=0)

  uint8_t dataHigh = (uint8_t)((data >> 8) & 0x0f); // get 4 msb of data
  uint8_t dataLow = (uint8_t)data;                  // low byte contains 8 lsb

  uint8_t reg = (uint8_t)(DAC_MODE_FAST << 6) | dataHigh; // DAC fast write mode, normal

  I2C1_Start();
  I2C1_WaitStartGen();

  I2C1->DR = addr; // clears START status bit
  if (I2C1_WaitAddrGen() == -1)
  {
    return;
  }

  I2C1_WaitTXEmpty();
  I2C1->DR = reg; // write dac register and 4 msb of data

  I2C1_WaitTXEmpty();
  I2C1->DR = dataLow; // write low byte of data

  I2C1_WaitByteTransfer();
  I2C1_Stop();
}

void TIM2_Init()
{
  // enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // edge-aligned, count up, no clock division, counter disabled
  TIM2->CR1 = 0;
  TIM2->CR2 = 0;

  TIM2->PSC = 10-1; // freq = 1.6 MHz

  TIM2->DIER = TIM_DIER_UIE; // update interrupt enable

  // To generate 1 Hz sine wave
  // 1.6 MHz timer clock ---> 6.25e-7s period
  // period * ARR * 4096 = 1 Hz
  // ARR = 1 / (period * 4096)
  TIM2->ARR = 390;       // TOP value --> interrupt generated every 1/freq * ARR
  
  TIM2->CR1 = TIM_CR1_ARPE; // auto preload enable
  TIM2->EGR = TIM_EGR_UG;   // re-initialize counter and an update registers
}

void TIM2_Start()
{
  TIM2->CNT = 0;             // reset counter
  NVIC_EnableIRQ(TIM2_IRQn); // enable interrupt on TIM2

  TIM2->CR1 |= TIM_CR1_CEN; // enable counter
}

void TIM2_IRQHandler()
{

  if (TIM2->SR & TIM_SR_UIF) // if interrupt was from update
  {

    angle += step;

    TIM2->SR &= ~TIM_SR_UIF; // interrupt flag is cleared by writing 0
  }
}

uint16_t convertSinToVoltage(double temp)
{

  // maps a sin value (-1 to 1) to a DAC voltage level (0 to 4096)
  const double oldMin = -1;    // sin(3pi/2)
  const double oldMax = 1;     // sin(pi/2)
  const double newMin = 0;     // DAC min voltage
  const double newMax = 4095;  // DAC max voltage
  const double oldRange = oldMax - oldMin;
  const double newRange = newMax - newMin;

  return (((temp - oldMin) * newRange) / oldRange) + newMin;
}
