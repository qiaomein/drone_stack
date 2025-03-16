/*
 * VL53L0X.h
 *
 *  Created on: Mar 13, 2025
 *      Author: qiaomein
 */

#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

#include "stm32l4xx_hal.h" // needed for I2C
#include "main.h"



#define VL53L0X_I2C_ADDR		(0x52) // in p.3 of datasheet
// must leftshift since I2C addr is 7 bit and the LSB is the R/W bit


// define the REFERENCE REGISTERS p. 18

#define VL53L0X_I2C_RR1_ADDR		(0xC0)
#define VL53L0X_I2C_RR2_ADDR		0xC1
#define VL53L0X_I2C_RR3_ADDR		0xC2
#define VL53L0X_I2C_RR4_ADDR		0x51
#define VL53L0X_I2C_RR5_ADDR		0x61

#define VL53L0X_I2C_RR1_READ		0xEE
#define VL53L0X_I2C_RR2_READ		0xAA // 8 bit
#define VL53L0X_I2C_RR3_READ		0x10
#define VL53L0X_I2C_RR4_READ		0x0099 // 16 bit readings
#define VL53L0X_I2C_RR5_READ		0x0000 // 16 bit


// these registers are from ST API so not in any pdf documentation
#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)


// registers for range measuring ?
#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

// constants for DAQ
#define VL53L0X_OUT_OF_RANGE (8190)


// struct of sensor data and properties

typedef struct {
	I2C_HandleTypeDef* i2c_handle;

	uint16_t distance;

} VL53L0X;


// START FUNCTION DEFINITIONS!!

HAL_StatusTypeDef VL53L0X_Init(VL53L0X* sensor, I2C_HandleTypeDef* i2c_handle);

HAL_StatusTypeDef VL53L0X_MeasureSingleDistance(VL53L0X* sensor);

// lower level functions


HAL_StatusTypeDef VL53L0X_ReadRegister(VL53L0X* sensor, uint8_t reg, uint8_t* data); //data is a pointer to the where data wil be read to

HAL_StatusTypeDef VL53L0X_ReadRegisters(VL53L0X* sensor, uint8_t reg, uint8_t* data, uint8_t length); //data is a pointer to the where data wil be read to


HAL_StatusTypeDef VL53L0X_WriteRegister(VL53L0X* sensor, uint8_t reg, uint8_t* data);



#endif /* INC_VL53L0X_H_ */
