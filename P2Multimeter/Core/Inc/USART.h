/*
* UART.h
*
* Created on: Nov 1, 2024
* Author: USER
*/
#ifndef INC_USART_H_
#define INC_USART_H_
#include "stm32l4xx_hal.h"
#define USART_INTERRUPT_EN 0x1F
#define AF 0x2
#define USART_AF 0x7
#define BR_DIV 208 // 24MHz/115.2kbps
#define ESC_KEY 0x1b
void USART_init(void);
void UART_print(char* string);
char* print_stat(int32_t input);
void UART_ESC_Code(char *string);
#endif /* INC_USART_H_ */
