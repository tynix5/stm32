
#include "uart2.h"
#include "stm32f4xx.h"

#define F_CLK				16000000
#define GPIOA_EN			(1U << 0)
#define USART2_EN			(1U << 17)
#define AFRL2				8
//#define USART_SR_TXE		(1U << 7)
//#define USART_CR1_UE		(1U << 13)
//#define USART_CR1_TE		(1U << 3)
//#define USART_CR2_CLKEN		(1U << 11)

void uart2_init(uint32_t baud) {

	/* Enable USART2 peripheral clock */
	RCC->APB1ENR |= USART2_EN;

	/* Enable GPIOA clock */
	RCC->AHB1ENR |= GPIOA_EN;

	/* Select alternate function (MODER = 10) for TX(PA2) pin */
	GPIOA->MODER |= (1 << 5);
	GPIOA->MODER &= ~(1 << 4);

	/* Select alternate function mode 7 for TX pin */
	GPIOA->AFR[0] &= ~(0xf << AFRL2);		// reset all bits for pin 2
	GPIOA->AFR[0] |= (0x7 << AFRL2);		// AF07


	/* Enable transmitter, Set word length: 1 start bit, 8 data bits, 0 stop bits */
	USART2->CR1 = USART_CR1_TE;

	/* Disable serial mode (CLK) */
	USART2->CR2 &= ~USART_CR2_CLKEN;

	/* Set baud rate */
	uart2_setbaud(baud);

	/* Enable UART2 */
	USART2->CR1 |= USART_CR1_UE;
}


void uart2_setbaud(uint32_t baud) {

	USART2->BRR = (F_CLK + (baud / 2U)) / baud;		// formula for baud in datasheet
}


void uart2_writebyte(uint8_t byte) {

	while (!(USART2->SR & USART_SR_TXE));		// wait until transmit data register is empty...

	USART2->DR = byte;			// send data, clearing TXE flag

}
