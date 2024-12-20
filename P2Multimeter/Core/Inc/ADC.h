/*
* ADC.h
*
* Created on: Nov 13, 2024
* Author: USER
*/
#ifndef INC_ADC_H_
#define INC_ADC_H_
#include "stm32l4xx_hal.h"
#define COMP_INTERRUPT 0x1F
#define ARR_VAL 5
#define CLKFREQ 24000000 // 24MHz clock
#define OFF 0
#define PRSCL 2400 // 10kHz clock to help 2kHz sampling
#define CHANNEL5 5
#define INIT_DELAY 50000// 20 us delay for ADC init
#define LONG_DELAY 200000 // Long enough for us to see the USART terminal
#define SLOPE ((float)0.8107)
#define OFFSET 0
#define ADC_CONV(actual) ((float)((actual*SLOPE)+OFFSET))
void ADC_init(void);
void ADC_delay(void);
#endif /* INC_ADC_H_ */
