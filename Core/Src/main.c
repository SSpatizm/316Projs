/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

void SystemClock_Config(void);

static Accel accel_list[FILTER_SIZE];
static Accel calibrated_offset;
static Accel avgAccel;

static Accel velocity;  //maybe this will do better as a float
static Accel gamePos;
static Accel goalPos;
static uint8_t lastDistance;
static uint8_t accel_index = 0;
static Reference globalRef;  //global initial pitch and roll for the acclerometer
static Position currPos;
static uint8_t movement_detected = 0;
static uint8_t current_movement = 0; //0 = no motion 1 = left(+x) 2 = right (-x) 3 = up(-y) 4 = down(+y)
static uint8_t last_movement = 1;
static uint8_t score = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  USART_init();
//  I2C_init();
//  IMU_init();
  GUI_init();
  TIM2_init();
  set_new_goal();
  delay_us(SEC_DELAY); // 1.5 s delay to calibrate sensor
  find_moving_average();
  calibrated_offset = recalibrate_sensor(avgAccel);
  globalRef = define_reference(avgAccel);

  while (1)
  {
	  static uint8_t heat;
	  find_moving_average();
	  calibrate_average();
	  orient_axis();
	  findPosition();
	  //check if goal reached
	  if(movement_detected && current_movement) {
		  uint8_t currDistance = check_distance(gamePos, goalPos);
		  if((gamePos.x == goalPos.x) && (gamePos.y == goalPos.y))
		  {
			  //you win!
			  heat = GOAL_FOUND;   //very hot
			  score++;
		  }
		  else if(currDistance < lastDistance) {
			  heat = HOT; //hotter
		  }
		  else {
			  heat = COLD;  //cold :(
		  }
		  lastDistance = currDistance;
		  movement_detected = 0;  //reset flag
		  printStatus(heat);
		  //reset goal if goal reached and whatnot
		  if(heat == GOAL_FOUND) {
			  set_new_goal();
			  heat = COLD;
	  }
	 }
  }
}

void TIM2_IRQHandler(void) {	// sample waveform to achieve 2khz sampling
  	if(TIM2->SR & TIM_SR_UIF){
  		//interrupt
  		if(!movement_detected) {
  		if(current_movement != last_movement) { //check if person is still moving in the same direction
			switch(current_movement)
			{
				case 0:
					break;
				case RIGHT:
					gamePos.x++;
					break;
				case LEFT:
					gamePos.x--;
					break;
				case DOWN:
					gamePos.y--;
					break;
				case UP:
					gamePos.y++;
					break;
				default:
					break;
			}
  		}
  		//reset position and velocity for next measurements.
  		current_movement = 0;
  		last_movement = current_movement;
  		movement_detected = 1;
  		}
  		TIM2->SR &= ~TIM_SR_UIF;
  	}
  }

void printStatus(uint8_t heat) {
	UART_ESC_Code("[17;19H");
	UART_print("Controls: Tilt in the direction you would like to go! You are moving: ");
	UART_ESC_Code("[0K");
	switch(current_movement)
	{
	case RIGHT:
		UART_print("Left.");   //Accelerometer is generally held upside down.
		break;
	case LEFT:
		UART_print("RIght.");  // Real Left and Real Right are swapped.
		break;
	case DOWN:
		UART_print("Down.");
		break;
	case UP:
		UART_print("Up.");
		break;
	default:
		break;
	}
	UART_ESC_Code("[3;16H");   //line 1 column 0
	UART_print(IntToStr(score));
	UART_ESC_Code("[7;15H");
	switch(heat)
	{
		case COLD:
			UART_print("Colder...");
			break;
		case HOT:
			UART_print("Hotter...");
			break;
		case GOAL_FOUND:
			UART_print("GOAL!!!!!");
			break;
		default:
			UART_print("Colder...");
	}
	UART_ESC_Code("[8;12H");
	UART_print(IntToStr(gamePos.x));
	UART_ESC_Code("[9;12H");
	UART_print(IntToStr(gamePos.y));
	UART_ESC_Code("[H");   //reset
}

void find_moving_average(void) {
	while (1) {
	if(accelDataAvailible())
	{
		if(accel_index == FILTER_SIZE) {  //finds average using filter size. Also reduces sampling rate
			int32_t xSum = 0;
			int32_t ySum = 0;
			int32_t zSum = 0;
			for(uint8_t i = 0; i < FILTER_SIZE; i++) {
				xSum += accel_list[i].x;
				ySum += accel_list[i].y;
				zSum += accel_list[i].z;
			}
			avgAccel.x = xSum / FILTER_SIZE;
			avgAccel.y = ySum / FILTER_SIZE;
			avgAccel.z = zSum / FILTER_SIZE;
			accel_index = 0;
			break;
		}
		accel_list[accel_index] = IMU_read_accel();
		accel_index++;
	}
	}

}


void orient_axis(void) { //completely orients the axis based on calibrated roll and pitch values
	int16_t mag = (int16_t)calculate_magnitude(avgAccel);
	accelVector unitVector = normalize_acceleration(avgAccel, mag);
	Reference currentRef = define_reference(avgAccel);
	currentRef = find_reference_difference(globalRef, currentRef);
	unitVector = align_acceleration(unitVector, currentRef);
	avgAccel.x = unitVector.x * mag;
	avgAccel.y = unitVector.y * mag;
	avgAccel.z = unitVector.z * mag;


}

uint8_t accelDataAvailible(void) {
	uint8_t status = IMU_read(LSM6DSOX_STATUS); // Read STATUS_REG
    return (status & 0x01); // Check if XLDA is set
}

void calibrate_average(void) {
	avgAccel.x += calibrated_offset.x;
	avgAccel.y += calibrated_offset.y;
	avgAccel.z += calibrated_offset.z;

}

void findPosition(void) {
	int32_t mag = calculate_magnitude(avgAccel);
	accelVector unitVector = normalize_acceleration(avgAccel, mag);
	velocity.x += unitVector.x * mag * DT;
	velocity.y += unitVector.y * mag * DT;
	velocity.z += unitVector.z * mag * DT;
	if (mag > GAME_SENSITIVITY) {  //VERY IMPORTANT. DEFINES GAME SENSITIVITY.
	if (abs(velocity.x) > abs(velocity.y)) {
		if(velocity.x > 0) {
			current_movement = RIGHT;
		}
		else{
			current_movement = LEFT;
		}
	}
	else
	{
		if(velocity.y > 0) {
			current_movement = UP;
		}
		else {
			current_movement = DOWN;
		}
	}
	velocity.x = 0;
	velocity.y = 0;
	velocity.z = 0;
}
}

void set_new_goal(void) {
	goalPos.x = rand() % MAP_SIZE;
	goalPos.y = rand() % MAP_SIZE;


}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void GUI_init(void) {
	currPos.x = 0;
	currPos.y = 0;
	currPos.z = 0;
	velocity.x = 0;
	velocity.y = 0;
	velocity.z = 0;
	UART_ESC_Code("[H");
	UART_ESC_Code("[2J");
	UART_ESC_Code("[3;1H");
	UART_print("Current Score: ");
	UART_ESC_Code("[7;1H");
	UART_print("Current heat: ");
	UART_ESC_Code("[8;1H");
	UART_print("gamePos x: ");
	UART_ESC_Code("[9;1H");
	UART_print("gamePos y: ");
	UART_ESC_Code("[11;1H");
	UART_print("#############################################");
	UART_ESC_Code("[13;1H");
	UART_print("      //\\      ");
	UART_ESC_Code("[14;1H");
	UART_print("     //--\\     ");
	UART_ESC_Code("[15;1H");
	UART_print(" ___//----\\___       $$Welcome to Hot and Cold! You must control your cruiser to find the hidden treasure!$$");
	UART_ESC_Code("[16;1H");
	UART_print("    \\------//   ");
	UART_ESC_Code("[17;1H");
	UART_print("     \\____//        Move Accelerometer to Begin!");  //17, 18
	UART_ESC_Code("[18;1H");
	UART_print("       ||       ");
	UART_ESC_Code("[19;1H");
	UART_print("       ||       ");
	UART_ESC_Code("[20;1H");
	UART_print("       ||       ");
	UART_ESC_Code("[21;1H");
	UART_print("       ||       ");
	UART_ESC_Code("[22;1H");
	UART_print("      ====      ");
	UART_ESC_Code("[H");

}

void delay_us(const uint32_t time_us) {
		// set the counts for the specified delay
		SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
		SysTick->VAL = 0;                                      // clear the timer count
		SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);        // clear the count flag
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for the flag
	}

	/* Configure SysTick Timer for use with delay_us function. This will break
	 * break compatibility with HAL_delay() by disabling interrupts to allow for
	 * shorter delay timing.
	 */
	void SysTick_Init(void){
		SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |	       // enable SysTick Timer
						  SysTick_CTRL_CLKSOURCE_Msk);     // select CPU clock
		SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);      // disable interrupt,
														   // breaks HAL delay function
	}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
