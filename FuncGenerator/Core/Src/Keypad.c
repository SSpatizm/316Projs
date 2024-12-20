#include "Keypad.h"
static void Keypad_TurnOnCols(void) {
	GPIOC->ODR |= GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12;
}
static void Keypad_TurnOffCols(void) {
	GPIOC->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);
}
// Cols: PC10-12, Rows: PB13, 14, 15, 1
void Keypad_Init(void) {
	  // Turn on clock for GPIOB and GPIOC
	  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
	  // Configure cols (PC10-12) for output
	  GPIOC->MODER &= ~(GPIO_MODER_MODE10
			  | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	  GPIOC->MODER |=
			  (1 << GPIO_MODER_MODE10_Pos)
			  | (1 << GPIO_MODER_MODE11_Pos)
			  | (1 << GPIO_MODER_MODE12_Pos);
	  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10
			  | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10
			  | GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10
			  | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);
	  // Configure rows (PB13-15,1) for input
	  GPIOB->MODER &= ~(
			  GPIO_MODER_MODE13
			  | GPIO_MODER_MODE14
			  | GPIO_MODER_MODE15
			  | GPIO_MODER_MODE1
		  );
	  // Use pull down resistors on rows.
	  GPIOB->PUPDR &= ~(
			  GPIO_PUPDR_PUPD13
			  | GPIO_PUPDR_PUPD14
			  | GPIO_PUPDR_PUPD15
			  | GPIO_PUPDR_PUPD1
		  );
	  GPIOB->PUPDR |=
			  GPIO_PUPDR_PUPD13_1
			  | GPIO_PUPDR_PUPD14_1
			  | GPIO_PUPDR_PUPD15_1
			  | GPIO_PUPDR_PUPD1_1;
	  Keypad_TurnOnCols();
}
int8_t Keypad_CalculateButton(int8_t row, int8_t col) {
	if (row <= 2) {
		return row * 3 + col + 1;
	}
	// Special cases for bottom row
	if (col == 0) return 10;
	if (col == 1) return 0;
	if (col == 2) return 11;
	return -1;
}
int8_t Keypad_GetButton(void) {
	// Return early if no buttons are pressed
	if (!(GPIOB->IDR & (GPIO_IDR_ID13 | GPIO_IDR_ID14 | GPIO_IDR_ID15 | GPIO_IDR_ID1))) {
		return -1;
	}
	for (int8_t col = 0; col <= 2; col++) {
		// Turn on only the specific column being tested
		Keypad_TurnOffCols();
		GPIOC->ODR |= 1 << (GPIO_ODR_OD10_Pos + col);
		// Check if any buttons are pressed in the column
		int8_t row = -1;
		uint32_t row_pins = GPIOB->IDR;
		if (row_pins & GPIO_IDR_ID13) row = 0;
		if (row_pins & GPIO_IDR_ID14) row = 1;
		if (row_pins & GPIO_IDR_ID15) row = 2;
		if (row_pins & GPIO_IDR_ID1) row = 3;
		// If none are pressed, keep going
		if (row == -1) {
			continue;
		}
		// Turn on all columns before returning
		Keypad_TurnOnCols();
		return Keypad_CalculateButton(row, col);
	}
	// Turn on all columns before returning
	Keypad_TurnOnCols();
	return -1;
}
