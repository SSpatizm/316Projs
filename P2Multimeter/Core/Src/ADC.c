/*
* ADC.c
*
* Created on: Nov 13, 2024
* Author: USER
*/
#include "ADC.h"
void ADC_init(void) {
// Enable ADC clock
RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
// Set ADC clock to HCLK/1 synchronous mode
ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);
// Power up ADC voltage regulator
ADC1->CR &= ~(ADC_CR_DEEPPWD);
ADC1->CR |= (ADC_CR_ADVREGEN);
// Wait for ADC voltage regulator to stabilize
ADC_delay();
// Configure ADC for single-ended mode on channel 5 (PA0)
ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);
// Calibrate the ADC
ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
ADC1->CR |= ADC_CR_ADCAL; // Start calibration
while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to finish
// Enable ADC and wait for it to be ready
ADC1->ISR |= (ADC_ISR_ADRDY);
ADC1->CR |= (ADC_CR_ADEN);
while (!(ADC1->ISR & ADC_ISR_ADRDY));
// Set up ADC to sample channel 5 once in a sequence
ADC1->SQR1 = (CHANNEL5 << ADC_SQR1_SQ1_Pos);
ADC1->CFGR = 0;
// Configure sample time
ADC1->SMPR1 = ~(ADC_SMPR1_SMP5);
ADC1->SMPR1 = (7 << ADC_SMPR1_SMP5_Pos);
// Enable interrupts on end of conversion
ADC1->IER |= (ADC_IER_EOC);
ADC1->ISR |= (ADC_ISR_EOC);
// Configure GPIO for channel 5 PA0
RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
GPIOA->ASCR |= (GPIO_ASCR_ASC0); // Connect analog switch to ADC input
GPIOA->MODER &= ~(GPIO_MODER_MODE0);
GPIOA->MODER |= (GPIO_MODER_MODE0); // Analog mode
GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0); // Max speed
// Enable interrupt in NVIC
NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
}
void ADC_delay(void) { // Delay for ADC setup
for (uint32_t i = 0; i < INIT_DELAY; i++);
}
