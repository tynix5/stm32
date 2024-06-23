/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : I2C demo - Sine wave with MCP4725 DAC
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

// PB8 --> I2C1_SCL
// PB9 --> I2C1_SDA
// Max Freq 100 kHz

#define DAC_WRITE 0b1100000
#define DAC_READ 0b1100000

// MCP4725 Registers
#define DAC_FAST 0b000
#define DAC_WRITE_REG 0b010
#define DAC_WRITE_REG_EEPROM 0b011

void I2C1_Init();
inline void I2C1_Start();
inline void I2C1_Stop();
inline void I2C1_Ack();
inline void I2C1_Nack();
void I2C1_writeByte(uint8_t addr, uint8_t reg, uint8_t data);
void I2C1_write(uint8_t addr, uint8_t reg, uint8_t data[], uint8_t len);
uint8_t I2C1_readByte(uint8_t addr, uint8_t reg);

void DAC_Write(uint16_t data);

int main()
{

  I2C1_Init();

  while (1)
  {

    for (unsigned int i = 0; i < 4095; i++)
    {
      DAC_Write(i);
      for (volatile unsigned int j = 0; j < 1000; j++) {}

    }

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

  GPIOB->OTYPER = GPIO_OTYPER_OT8 |   // open drain on SCL
                  GPIO_OTYPER_OT9;    // open drain on SDA

  // Initialize I2C in standard mode (<100kHz)
  RCC->APB1ENR = RCC_APB1ENR_I2C1EN; // enable I2C clock

  I2C1->CR1 = 0;              // default configuration (I2C disabled)
  I2C1->CR2 = I2C_CR2_FREQ_1; // 2 MHz APB clock frequency

  I2C1->CCR = 100; // 20k baud, Sm mode (standard)

  I2C1->CR1 = I2C_CR1_PE; // enable peripheral
}

inline void I2C1_Start()
{
  // generate start condition, master mode
  I2C1->CR1 |= I2C_CR1_START;
}

inline void I2C1_Stop()
{
  // generate stop condition
  I2C1->CR1 |= I2C_CR1_STOP;
}

inline void I2C1_Ack()
{
  // generate acknowledge
  I2C1->CR1 |= I2C_CR1_ACK;
}

inline void I2C1_Nack()
{
  // generate no acknowledge
  I2C1->CR1 &= ~I2C_CR1_ACK;
}

void I2C1_writeByte(uint8_t addr, uint8_t reg, uint8_t data)
{
  // uint8_t rw = (uint8_t)addr << 1;

  // I2C1_Start();

  // while (!(I2C1->SR1 & I2C_SR1_TXE));   // wait until data register is empty
  // I2C1->DR = rw;    // write 7 bit address + write (LSB=0)
 
  // while (!(I2C1->SR1 & I2C_SR1_TXE));   // wait until data register is empty
  // I2C1->DR = data;

  // I2C1_Stop();
}

void I2C1_write(uint8_t addr, uint8_t reg, uint8_t data[], uint8_t len)
{
}

void DAC_Write(uint16_t data)
{

  uint8_t addr = (uint8_t) (DAC_WRITE << 1);      // 7 bit address + write (LSB=0)
  uint8_t reg = (uint8_t) (DAC_WRITE_REG << 5);   // DAC register + normal mode

  uint8_t dataLow = (uint8_t) (data << 4);    // low byte contains 4 LSB
  uint8_t dataHigh = (uint8_t) (data >> 4);   // high byte contains 8 MSB

  I2C1_Start();

  while (!(I2C1->SR1 & I2C_SR1_SB));    // wait for start condition generated status bit

  I2C1->DR = addr;    // clears TXE status bit
  while (!(I2C1->SR1 & I2C_SR1_ADDR));   // wait until address is sent
  volatile uint8_t temp = I2C1->SR2;  // read SR2 to clear ADDR status bit

  while (!(I2C1->SR1 & I2C_SR1_TXE));   // wait until data register is empty
  I2C1->DR = reg;    // write dac register

  while (!(I2C1->SR1 & I2C_SR1_TXE));   // wait until data register is empty
  I2C1->DR = dataHigh;    // write high byte of DAC

  while (!(I2C1->SR1 & I2C_SR1_TXE));   // wait until data register is empty
  I2C1->DR = dataLow;    // write low byte of DAC

  while (!(I2C1->SR1 & I2C_SR1_TXE) && !(I2C1->SR1 & I2C_SR1_BTF));     // wait until data register empty and byte transfer complete

  I2C1_Stop();
}