/*
 * Keypad.h
 *
 *  Created on: Oct 27, 2024
 *      Author: user
 */

#ifndef SRC_KEYPAD_H_
#define SRC_KEYPAD_H_
#include <stdint.h>
#include "stm32l4xx_hal.h"
static void Keypad_TurnOnCols(void);
static void Keypad_TurnOffCols(void);
void Keypad_Init(void);
int8_t Keypad_GetButton(void);


#endif /* SRC_KEYPAD_H_ */
