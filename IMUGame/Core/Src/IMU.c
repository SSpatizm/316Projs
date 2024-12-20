#include "IMU.h"

void I2C_init(void)
	{
		// Init GPIO PB10 (SCL) yellow and PB11 (SDA) blue for I2C
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

				// AF and OD, no PUPD, max output speed
		GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
		GPIOB->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1;

		GPIOB->OTYPER |= GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11;
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
		//the IMU itself already has internal pullups
		GPIOB->AFR[1] |= (AF4 << GPIO_AFRH_AFSEL10_Pos) | (AF4 << GPIO_AFRH_AFSEL11_Pos); //AFRL, AF4 for I2C
		GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11;

				// Init I2C for general use
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
		I2C2->CR1 &= ~I2C_CR1_PE;
				// I2C Timing setup
		I2C2->TIMINGR = I2C1TIMING;	//0xE14	for the frequency division
				// ENABLE I2C
		I2C2->CR1 |= I2C_CR1_PE;
	}

void IMU_init(void) {
    // verify WHO_AM_I register
    uint8_t verify = IMU_read(LSM6DSOX_WHO_AM_I);
    if (verify != LSM6DSOX_WHO_ARE_YOU) {
        while (1); // Device not responding correctly, PAUSE!
    }
    // Reset the IMU
       IMU_write(LSM6DSOX_CTRL3_C, 0x01);
       delay_us(DELAY_MS); // Wait for reset to complete (100 ms)
       // Configure accelerometer: 3.3kHz Hz ODR, ±2g range(old: 0x96 on hi perf)
       IMU_write(LSM6DSOX_CTRL1_XL, 0x62);
       // Configure gyroscope: 208 Hz ODR, ±2000 dps range
       IMU_write(LSM6DSOX_CTRL2_G, 0x4C);
       // Enable low-pass filter for accelerometer
       IMU_write(LSM6DSOX_CTRL8_XL, 0x89);
       // Enable X, Y, Z axes for accelerometer
       IMU_write(LSM6DSOX_CTRL9_XL, 0x38);
       // Enable accelerometer and gyroscope in the master control register
       IMU_write(LSM6DSOX_CTRL10_C, 0x38);
    delay_us(DELAY_MS); // 100 ms delay
}

Accel recalibrate_sensor(Accel avgAccel) {
	Accel offset;
    offset.x = -avgAccel.x;
    offset.y = -avgAccel.y;
    offset.z = -avgAccel.z;
    return offset;


}


Accel IMU_read_accel(void) {
	Accel accel;
	// x
	uint8_t xLo = IMU_read(LSM6DSOX_OUTX_L_A);
	uint8_t xHi = IMU_read(LSM6DSOX_OUTX_H_A);
	accel.x = (int16_t)((xHi << BYTE_SIZE) | xLo) * ACCEL_SENS / GRAVITY;
	// y
	uint8_t yLo = IMU_read(LSM6DSOX_OUTY_L_A);
	uint8_t yHi = IMU_read(LSM6DSOX_OUTY_H_A);
	accel.y = (int16_t)((yHi << BYTE_SIZE) | yLo) * ACCEL_SENS / GRAVITY;
	// z
	uint8_t zLo = IMU_read(LSM6DSOX_OUTZ_L_A);
	uint8_t zHi = IMU_read(LSM6DSOX_OUTZ_H_A);
	accel.z = (int16_t)((zHi << BYTE_SIZE) | zLo) * ACCEL_SENS / GRAVITY;

	return accel;

}



uint8_t IMU_read(uint8_t regAddy) {

	//write mode
	I2C2->CR2 = (I2C_CR2_AUTOEND | (IMU_I2C_ADDRESS << 1) | (1 << I2C_CR2_NBYTES_Pos));
	I2C2->CR2 |= I2C_CR2_START;
	while(!(I2C2->ISR & I2C_ISR_TXIS));	// Wait for control byte ready
	I2C2->TXDR = regAddy;
	while(!(I2C2->ISR & I2C_ISR_STOPF));		// Address written

	//read mode
	I2C2->CR2 = (I2C_CR2_AUTOEND | (IMU_I2C_ADDRESS << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN);
	I2C2->CR2 |= I2C_CR2_START;
	while (!(I2C2->ISR & I2C_ISR_RXNE));
    uint8_t data = I2C2->RXDR;
	while(!(I2C2->ISR & I2C_ISR_STOPF));		// Address written

    I2C2->ICR = I2C_ICR_STOPCF; //clear stop flag

    return data;
}

void IMU_write(uint8_t regAddy, uint8_t value) {

	// Set CR2 register for writing and start
	I2C2->CR2 = (I2C_CR2_AUTOEND | (IMU_I2C_ADDRESS << 1) | (2 << I2C_CR2_NBYTES_Pos)); //2 bytes

	I2C2->CR2 |= I2C_CR2_START;

	while(!(I2C2->ISR & I2C_ISR_TXE));	// Wait for control byte
    I2C2->TXDR = regAddy;

	while (!(I2C2->ISR & I2C_ISR_TXIS));  // Wait for transmit empty

	I2C2->TXDR = value;
	while(!(I2C2->ISR & I2C_ISR_STOPF));		// Write data and wait for transmit to be complete
}

int32_t calculate_magnitude(Accel accel) {
	return sqrt((accel.x * accel.x)+(accel.y * accel.y)+(accel.z * accel.z));
}

uint8_t check_distance(Accel pos1, Accel pos2) {
	int16_t dx = (pos2.x - pos1.x) * BYTE_SIZE;   //increase by byte size for higher accuracy
	int16_t dy = (pos2.y - pos1.y) * BYTE_SIZE;
	return (uint8_t)sqrt((dx * dx) + (dy * dy));
}

accelVector normalize_acceleration(Accel accel, int32_t mag) {
	accelVector normalized;
	if(mag > 0) {
		normalized.x = (float) accel.x/ mag;
		normalized.y = (float) accel.y/ mag;
		normalized.z = (float) accel.z/ mag;
	}
	else {
		normalized.x = 0;
		normalized.y = 0;
		normalized.z = 0;
	}
	return normalized;
}

accelVector align_acceleration(accelVector normAccel, Reference globalRef) {
	//MATH MATH MATH ROATATION MATRIX VECTOR CACLULATION MATRIX MULTIPLICATION COMPUTATION POWER!
	accelVector gps;
	float pitch = globalRef.pitch;
	float roll = globalRef.roll;

	//rotation matrices. I might cry. Make sure to add the aero stuff to references
	//https://www.cuemath.com/algebra/rotation-matrix/
	float cos_pitch = cos(pitch);
	float sin_pitch = sin(pitch);
	float cos_roll = cos(roll);
	float sin_roll = sin(roll);

	//whole a** matrix. bro
	float r00 = cos_roll;
	float r01 = -sin_roll * sin_pitch;
	float r02 = sin_roll * cos_pitch;
	float r10 = 0;
	float r11 = cos_pitch;
	float r12 = sin_pitch;
	float r20 = -sin_roll;
	float r21 = -cos_roll * sin_pitch;
	float r22 = cos_roll * cos_pitch;
	//perform matrix multiplication. Special thanks to cuemath.com
	gps.x = (r00 * normAccel.x + r01 * normAccel.y + r02 * normAccel.z);
	gps.y = (r10 * normAccel.x + r11 * normAccel.y + r12 * normAccel.z);
	gps.z = (r20 * normAccel.x + r21 * normAccel.y + r22 * normAccel.z);

	return gps;
}

Reference define_reference(Accel avgAccel) {
    uint32_t magnitude = calculate_magnitude(avgAccel);
    accelVector normalized = normalize_acceleration(avgAccel, magnitude);
    Reference globalRef;

    globalRef.pitch = asin(normalized.x);  // Initial pitch angle
    globalRef.roll = atan2(normalized.y, normalized.z);  // Initial roll angle
    return globalRef;
}

Reference find_reference_difference(Reference globalRef, Reference currentRef) {
	Reference difference;
	difference.pitch = currentRef.pitch - globalRef.pitch;
	difference.roll = currentRef.roll - globalRef.roll;
	return difference;
}
