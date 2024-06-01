#ifndef __UART2_H__
#define __UART2_H__

#include "stdint.h"

void UART2_Init(uint32_t baud);
void UART2_SetBaud(uint32_t baud);
void UART2_WriteChar(uint8_t byte);

#endif
