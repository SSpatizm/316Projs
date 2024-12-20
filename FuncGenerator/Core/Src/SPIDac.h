/*
* spi_dac.h
*
*  Created on: Oct 17, 2024
*      Author: pengu
*/
#ifndef SRC_SPI_DAC_H_
#define SRC_SPI_DAC_H_
#include <stdint.h>
#include "stm32l4xx_hal.h"
#define DAC_DACA_Pos 0xF // DAC_A/B bit in DAC data packet (bit 15)
#define DAC_BUFF_Pos 0xE // BUFF bit in DAC data packet (bit 14)
#define DAC_GA_Pos 0xD 	// GA bit in DAC data packet (bit 13)
#define DAC_SHDN_Pos 0xC	// SHDN bit in DAC data packet (bit 12)
#define GPIO_DATA_Mask 0x0FFF
void spi_setup();
void send_dac(uint16_t in_volt);
#endif /* SRC_SPI_DAC_H_ */
