/*
 * IMU.c
 *
 *  Created on: May 1, 2025
 *      Author: qiaomein
 */





#include "IMU.h"


HAL_StatusTypeDef IMU_Init(IMU* sensor, I2C_HandleTypeDef* i2c_handle){

	sensor->i2c_handle = i2c_handle;

	uint8_t data = 0b00000000;
	IMU_WriteRegister(sensor, REG_BANK_SEL, &data);
	HAL_Delay(50);
	//IMU_WriteRegister(sensor, USER_CTRL, &data);


	IMU_ReadRegister(sensor, WHO_AM_I, &data);
	if (data != WHO_AM_I_READ){
		return HAL_ERROR;
	}

	// Wake up the sensor
	uint8_t pwr_mgmt_1 = 0x01;  // Set CLKSEL=1 (Auto selects best available clock source)
	IMU_WriteRegister(sensor, PWR_MGMT_1, &pwr_mgmt_1);

	// Enable accel & gyro (disable sleep)
	uint8_t pwr_mgmt_2 = 0x00;
	IMU_WriteRegister(sensor, PWR_MGMT_2, &pwr_mgmt_2);



	return HAL_OK;
}


void IMU_GetAccel(IMU* sensor, int16_t* accels){ //accels is array of size 3
	// ax,ay,az = accels


//	uint8_t hdata,ldata;
//
//
//	IMU_ReadRegister(sensor, ACCEL_XOUT_H, &hdata);
//	IMU_ReadRegister(sensor, ACCEL_XOUT_L, &ldata);
//
//	accels[0] = (hdata << 8) + ldata; // ax


	uint8_t buffer[6];
	IMU_ReadRegisters(sensor, ACCEL_XOUT_H, buffer, 6);
	accels[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
	accels[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
	accels[2] = (int16_t)((buffer[4] << 8) | buffer[5]);

	return;

}


// READING AND WRITING REGISTERS

HAL_StatusTypeDef IMU_ReadRegister(IMU* sensor, const uint8_t reg, uint8_t* pData) {
	//data is a pointer to the where data will be read to

	return HAL_I2C_Mem_Read(sensor->i2c_handle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);

}



HAL_StatusTypeDef IMU_ReadRegisters(IMU* sensor, const uint8_t reg, uint8_t* pData, const uint8_t length) {
	//data is a pointer to the where data will be read to

	return HAL_I2C_Mem_Read(sensor->i2c_handle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef IMU_WriteRegister(IMU* sensor, const uint8_t reg, uint8_t* pData){

	return HAL_I2C_Mem_Write(sensor->i2c_handle, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);

}
