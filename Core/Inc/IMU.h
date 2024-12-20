/*
 * IMU.h
 *
 *  Created on: Nov 28, 2024
 *      Author: user
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32l4xx_hal.h"
#include "main.h"
#include "math.h"
//#include "arm_math.h"
//#include "float.h"


#define AF4 4              // Alternate function 4
#define I2C1TIMING 0x0E14 // I2C timing for 4MHz clock and I2C1 in normal mode(62 - 100kHz) i2C
#define IMU_I2C_ADDRESS 0x6A  // Base I2C address for the adafruit IMU

#define LSM6DSOX_CTRL1_XL     0x10  // Register for accelerometer control
#define LSM6DSOX_CTRL2_G      0x11  //register for gyroscope control
#define LSM6DSOX_CTRL3_C      0x12  // Control register for reset and enabling
#define LSM6DSOX_CTRL6_C      0x15  //control registeroperating modes
#define LSM6DSOX_CTRL8_XL     0x17  //LPF filtet
#define LSM6DSOX_CTRL9_XL     0x18
#define LSM6DSOX_CTRL10_C     0x19

#define LSM6DSOX_WHO_AM_I     0x0F  // WHO_AM_I register address
#define LSM6DSOX_WHO_ARE_YOU 0x6C  // Expected value for LSM6DSOX WHO_AM_I
#define LSM6DSOX_STATUS 0x1E   //status register

#define LSM6DSOX_OUTX_L_A  0x28  // x-axis low byte
#define LSM6DSOX_OUTX_H_A  0x29  // x-axis high byte
#define LSM6DSOX_OUTY_L_A  0x2A  // y-axis low byte
#define LSM6DSOX_OUTY_H_A  0x2B  // y-axis high byte
#define LSM6DSOX_OUTZ_L_A  0x2C  // z-axis low byte
#define LSM6DSOX_OUTZ_H_A  0x2D  // z-axis high byte

#define ACCEL_SENS 0.15f
#define GRAVITY 9.81f
#define PI 3.1415f
#define DT 0.3f
#define DELAY_MS 100000
#define BYTE_SIZE 8

typedef struct {
	int16_t x;  //can be negative too, 2 bytes of data
	int16_t y;
	int16_t z;
}Accel;

typedef struct { //this one is a unit vector, and thus requires floats
	float x;
	float y;
	float z;
}accelVector;


typedef struct {
    float pitch; // pitch angle
    float roll;  // roll angle
    float yaw;   // yaw angle
} Reference;

int32_t calculate_magnitude(Accel accel);

void I2C_init(void);
void IMU_init(void);
uint8_t IMU_read(uint8_t regAddy);
void IMU_write(uint8_t regAddy, uint8_t value);
Accel recalibrate_sensor(Accel accelAvg);
Accel IMU_read_accel(void);
accelVector normalize_acceleration(Accel accel, int32_t mag);
accelVector align_acceleration(accelVector normAccel, Reference globalRef);
Reference define_reference(Accel avgAccel);
Reference find_reference_difference(Reference globalRef, Reference currentRef);
uint8_t check_distance(Accel pos1, Accel pos2);



#endif /* INC_IMU_H_ */
