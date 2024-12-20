/*
 * TIM2.c
 *
 *  Created on: Nov 19, 2024
 *      Author: user
 */
#include "TIM2.h"

void TIM2_init(void) {
    // TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    // Clock frequency is 24MHz / PSC to equal 10kHz counter freq(PSC = 2400)
    TIM2->PSC = PRSCL - 1;
    // Set ARR = 5 for 2kHz
    TIM2->ARR = ARR_VAL - 1;
    // Enable update interrupt and disable CCR interrupt
    TIM2->DIER |= (TIM_DIER_UIE);
    TIM2->DIER &= ~TIM_DIER_CC1IE;
    // Clear flag
    TIM2->SR &= ~(TIM_SR_UIF);
    // Count up mode
    TIM2->CR1 &= ~(TIM_CR1_DIR);
    // Enable TIM2 interrupt in NVIC
    NVIC->ISER[0] = (1 << TIM2_IRQn);
    // Enable global interrupts
    __enable_irq();
    // Enable timer
    TIM2->CR1 |= TIM_CR1_CEN;
}
