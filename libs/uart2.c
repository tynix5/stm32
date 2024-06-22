
#include "uart2.h"
#include "stm32f4xx.h"

#define F_CLK				16000000
#define AFRL2				8			// alternate function mode selection position


void UART2_Init(uint32_t baud) {

	// Enable USART2 peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// Enable GPIOA clock 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Select alternate function (MODER = 10) for TX(PA2) pin
	GPIOA->MODER |= (1 << 5);
	GPIOA->MODER &= ~(1 << 4);

	// Select alternate function mode 7 for TX pin
	GPIOA->AFR[0] &= ~(0xf << AFRL2);		// reset all bits for pin 2
	GPIOA->AFR[0] |= (0x7 << AFRL2);		// AF07


	// Enable transmitter, Set word length: 1 start bit, 8 data bits, 0 stop bits
	USART2->CR1 = USART_CR1_TE;

	// Disable serial mode (CLK)
	USART2->CR2 &= ~USART_CR2_CLKEN;

	// Set baud rate
	UART2_SetBaud(baud);

	// Enable UART2
	USART2->CR1 |= USART_CR1_UE;
}


void UART2_SetBaud(uint32_t baud) {

	USART2->BRR = (F_CLK + (baud / 2U)) / baud;		// formula for baud in datasheet
}


void UART2_WriteChar(uint8_t byte) {

	while (!(USART2->SR & USART_SR_TXE));		// wait until transmit data register is empty...

	USART2->DR = byte;			// send data, clearing TXE flag

}


/* printf() uses __io_putchar() to print strings */
/* Override __io_putchar (defined as weak) so that the characters
 * are redirected to UART2 port
 */
int __io_putchar(int ch) {

	UART2_WriteChar((uint8_t)ch);			// use uart2 to send character
	return ch;
}
