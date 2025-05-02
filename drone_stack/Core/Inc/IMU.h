/*
 * IMU.h
 *
 *  Created on: May 1, 2025
 *      Author: qiaomein
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_


#include "stm32l4xx_hal.h" // needed for I2C
#include "main.h"


// REGISTERS

#define IMU_I2C_ADDR		(0b1101001 << 1) //p.28 of datasheet; 7 bit, the 8th bit is r/w (1/0) bit
#define WHO_AM_I			(0x00) //p. 32 for register maps
#define WHO_AM_I_READ		(0XEA)
#define USER_CTRL			(0x03)
#define REG_BANK_SEL		(0x7F)
#define PWR_MGMT_1			(0x06)
#define PWR_MGMT_2			(0x07)

#define ACCEL_XOUT_H		(0x2D)
#define ACCEL_XOUT_L		(0x2E)
#define ACCEL_YOUT_H		(0x2F)
#define ACCEL_YOUT_L		(0x30)
#define ACCEL_ZOUT_H		(0x31)
#define ACCEL_ZOUT_L		(0x32)


typedef struct {
	I2C_HandleTypeDef* i2c_handle;
	// TODO: accel[3], gyro[3], mag[3]
} IMU;



HAL_StatusTypeDef IMU_Init(IMU* sensor, I2C_HandleTypeDef* i2c_handle);
void IMU_GetAccel(IMU* sensor, int16_t* accels);
void IMU_GetGyro(IMU* sensor, int16_t* gyros);

// lower level functions

HAL_StatusTypeDef IMU_ReadRegister(IMU* sensor, uint8_t reg, uint8_t* data); //data is a pointer to the where data wil be read to

HAL_StatusTypeDef IMU_ReadRegisters(IMU* sensor, uint8_t reg, uint8_t* data, uint8_t length); //data is a pointer to the where data wil be read to

HAL_StatusTypeDef IMU_WriteRegister(IMU* sensor, uint8_t reg, uint8_t* data);

#endif /* INC_IMU_H_ */
