#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_hal.h"
#include "IMU.h"
#include "USART.h"
#include "math.h"
#include "stdlib.h"
#include "TIM2.h"

#define FILTER_SIZE 20
#define noise_removal 0
#define MAP_SIZE 10;
#define GAME_SENSITIVITY 30
#define COLD 0
#define HOT 1
#define GOAL_FOUND 2
#define SEC_DELAY 150000

#define UP    4
#define DOWN  3
#define LEFT  2
#define RIGHT 1


typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
}Position;

void Error_Handler(void);

void SysTick_Init(void);
void LED_init(void);
void GUI_init(void);

void delay_us(const uint32_t time_us);
void printStatus(uint8_t heat);
uint8_t accelDataAvailible(void);
void find_moving_average(void);
void orient_axis(void);
void findPosition(void);
void calibrate_average(void);

void check_last_movement(void);
void set_new_goal(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
