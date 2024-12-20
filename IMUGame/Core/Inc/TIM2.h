/*
 * TIM2.h
 *
 *  Created on: Nov 19, 2024
 *      Author: user
 */

#ifndef SRC_TIM2_H_
#define SRC_TIM2_H_
#include "stm32l4xx_hal.h"


// Timer stuff
#define PRSCL 400
#define ARR_VAL 7500
void TIM2_init(void);


#endif /* SRC_TIM2_H_ */
