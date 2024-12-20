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

static uint8_t conv_flag = 0;
static uint16_t ADC_sample;

static int16_t samples[SAMPLE_SIZE];
static uint16_t sample_index = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  USART_init();
  Meter_init();
  TIM2_init();
  ADC_init();
  while (1)
  {
	  // If ADC flag was procced(end of calculations)

	  if (!conv_flag) {
		  printValues();
		  //allow conversions again
		  conv_flag = 1;
	  }
  }
}

void TIM2_IRQHandler(void) {	// sample waveform to achieve 2khz sampling

  	if(TIM2->SR & TIM_SR_UIF){
  		if(conv_flag){
  			// Start an ADC conversion only if done printing
  			ADC1->CR |= ADC_CR_ADSTART;
  		}
  		// Clear flag
  		TIM2->SR &= ~TIM_SR_UIF;
  	}
  }
  //	Store new conversion value into global register and flag
void ADC1_2_IRQHandler(void) {
  	// Check for end of calculation interrupt
  	if (ADC1->ISR & ADC_ISR_EOC) {
  		// update global sample value
  		ADC_sample = ADC1->DR;
  		samples[sample_index] = ADC_sample;
  		sample_index++;
  		if(sample_index == SAMPLE_SIZE) {
  					  //turn off flag, prepare to print values
  			conv_flag = 0;
  			sample_index = 0;
  		}
  		// clear interrupt flag by writing a 1
  		ADC1->ISR &= ~ADC_ISR_EOC;
  	}
}

void printValues(void) {
	  uint32_t sum = 0;
	  uint16_t min = MAX_BITS;
	  uint16_t max = 0;
	  uint64_t rms_sum = 0;
	  uint8_t bound = 0;
	  uint16_t passes = 0;

	  for (uint16_t i = 0; i < SAMPLE_SIZE; i++)
	  {
		  rms_sum += samples[i] * samples[i];
		  sum += samples[i];
		  if (samples[i] > max )
		  {
			  max = samples[i];
		  }
		  if (samples[i] < min )
		  {
			  min = samples[i];
		  }
	  }

	  uint16_t avg = sum/SAMPLE_SIZE;
	  uint16_t Vpp = max-min;

	  //RMS
	  uint16_t Vrms = sqrt(rms_sum / SAMPLE_SIZE);

	  //Frequency calculations
	  for(uint16_t i = 0; i < SAMPLE_SIZE; i++)
	  {
		  if((samples[i] > (avg + HYST)) && (bound == LOWER_BOUND)) {
			  passes++;
			  bound = 0;
		  }
		  if((samples[i] < (avg - HYST)) && (bound == UPPER_BOUND)) {
			  passes++;
			  bound = 1;
		  }
	  }
	  //works for sine, square, triangle, and saw
	  uint16_t freq = passes / 2;

	  printVac(Vrms, Vpp);
	  printVdc(avg);
	  printFreq(freq);
}

void Meter_init(void) {
	//To print to dc(bar): 10, 0
	//dc(#): 46, 3
	//To print to rms(bar): 10, 5
	//rms(#): 46, 8
	//To print to freq: 25, 8
	//To print to Vpp: 27, 9
	UART_ESC_Code(RST);
	UART_ESC_Code(CLR);
	UART_ESC_Code("[3B"); //down 3
	UART_ESC_Code("[10C"); //right 10
	UART_print("|-----|-----|-----|-----|-----|-----|-----|"); //horizontal bar
	UART_ESC_Code("[1B");  //down 1
	UART_ESC_Code("[42D");   //left 42
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0   3.5");  //legend
	UART_ESC_Code("[1B");  //down 1
	UART_ESC_Code("[22D"); //left 22
	UART_print("DC Voltage(V): ");
	UART_ESC_Code(RST);//create borders
	UART_print("|====================DC MEASUREMENTS===================|");
	UART_ESC_Code("[8;0H"); //Next Border
	UART_print("|====================AC MEASUREMENTS===================|");
	UART_ESC_Code(RST);
	UART_ESC_Code("[10B"); //down 10
	UART_ESC_Code("[10C"); //right 10
	UART_print("|-----|-----|-----|-----|-----|-----|-----|"); //horizontal bar
	UART_ESC_Code("[1B");  //down 1
	UART_ESC_Code("[42D");   //left 42
	UART_print("0    0.5   1.0   1.5   2.0   2.5   3.0   3.5"); //legend
	UART_ESC_Code("[1B");  //down 1
	UART_ESC_Code("[22D"); //left 22
	UART_print("RMS Voltage(V): ");
	UART_ESC_Code(RST);
	UART_ESC_Code("[14B");   //down 14
	UART_ESC_Code("[10C");   //right 10
	UART_print("Frequency(Hz):");
	UART_ESC_Code("[1B");    //down 1
	UART_ESC_Code("[14D");   //left 14
	UART_print("Peak to Peak Voltage(Vpp): ");
	UART_ESC_Code("[18;0H"); //Insert bottom bar
	UART_print("|======================================================|");
}

//prints out AC voltage values
void printVac(uint16_t Vrms, uint16_t Vpp) {
	UART_ESC_Code("[10;11H"); //move to line 10 column 11
	UART_ESC_Code("[0K");  //clear current bar
	uint8_t tickBar = ((Vrms * BAR_LENGTH) / MAX_BITS); //find ticks needed
	for(uint8_t i = 0; i < tickBar; i++) {
		UART_print("#");
	}
	UART_ESC_Code("[13;50H");  //move to line 13 column 50
	UART_print(print_stat(ADC_CONV(Vrms)));
	UART_ESC_Code("[16;38H");    //move to line 16 column 50
	UART_print(print_stat(ADC_CONV(Vpp)));
}

void printVdc(uint16_t Vdc) {
	UART_ESC_Code("[3;11H"); //beginning 11
	UART_ESC_Code("[0K");  //clear current bar
	uint8_t tickBar = ((Vdc * BAR_LENGTH) / MAX_BITS); //find ticks needed
	for(uint8_t i = 0; i < tickBar; i++) {
		UART_print("#");
	}
	UART_ESC_Code("[6;49H"); //move to line 6 column 49
	UART_print(print_stat(ADC_CONV(Vdc)));
}

void printFreq(uint16_t freq) {
	UART_ESC_Code("[15;26H"); //move to line 15 column 26
	UART_print(FreqToStr(freq));
}

char* FreqToStr(uint16_t freq) {
static char string[5];  //4 letters
string[0] = (freq / 1000) + '0'; 			// Thousands
string[1] = (find_unit(freq / 100)) + '0'; // Hundreds
string[2] = (find_unit(freq / 10)) + '0';  // Tens
string[3] = (find_unit(freq)) + '0'; 	 	// Ones
string[4] = '\0'; // Null terminator
return string;
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
