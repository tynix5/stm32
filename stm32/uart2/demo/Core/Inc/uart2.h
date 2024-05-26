#ifndef __UART2_H__
#define __UART2_H__

#include "stdint.h"

void uart2_init(uint32_t baud);
void uart2_setbaud(uint32_t baud);
void uart2_writebyte(uint8_t byte);

#endif
