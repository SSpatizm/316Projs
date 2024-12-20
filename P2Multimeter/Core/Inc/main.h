/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "math.h"
#include "USART.h"
#include "ADC.h"
#include "TIM2.h"

#define CLR "[2J"
#define RST "[H"
#define BAR_LENGTH 43
#define MAX_BITS 4095
#define SAMPLE_SIZE 2000
#define find_unit(digit) (digit % 10) // finds digit in 1s
#define HYST 0
#define LOWER_BOUND 1
#define UPPER_BOUND 0

void SystemClock_Config(void);
void printValues(void);
void Meter_init(void);
void printVac(uint16_t Vrms, uint16_t Vpp);
void printVdc(uint16_t Vdc);
void printFreq(uint16_t freq);
uint16_t volt_conv(uint16_t Vin);
char* FreqToStr(uint16_t freq);


void Error_Handler(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
