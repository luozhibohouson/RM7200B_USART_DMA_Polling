#ifndef __USART_H
#define __USART_H

#include "main.h"

#define ENABLE_USART   0

void USART_PrintfConfigure(uint32_t Baudrate);
void USART_Configure(uint32_t Baudrate);
void usart_callback(void);

#endif

