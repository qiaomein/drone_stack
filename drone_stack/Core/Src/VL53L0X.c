



#include "VL53L0X.h"

static uint8_t stop_variable;

// define a lil macro here hehe
void write_sensor_reg(VL53L0X* sensor, uint8_t reg, uint8_t val){
	VL53L0X_WriteRegister(sensor, reg, &val);
}



typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

static HAL_StatusTypeDef perform_single_ref_calibration(VL53L0X* sensor, calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    uint8_t data;


    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }


    if (VL53L0X_WriteRegister(sensor, REG_SYSTEM_SEQUENCE_CONFIG, &sequence_config) != HAL_OK) {
        return HAL_ERROR;
    }
    if (VL53L0X_WriteRegister(sensor, REG_SYSRANGE_START, &sysrange_start) != HAL_OK) {
		return HAL_ERROR;
	}


    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    HAL_StatusTypeDef status = HAL_ERROR;

    do {
        status = VL53L0X_ReadRegister(sensor, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while ((status == HAL_OK) && ((interrupt_status & 0x07) == 0));
    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    data = 0x01;

    if (VL53L0X_WriteRegister(sensor, REG_SYSTEM_INTERRUPT_CLEAR, &data) != HAL_OK) {
        return HAL_ERROR;
    }
    data = 0x00;
    if (VL53L0X_WriteRegister(sensor, REG_SYSRANGE_START, &data) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}


HAL_StatusTypeDef VL53L0X_Init(VL53L0X* sensor, I2C_HandleTypeDef* i2c_handle){
	sensor->i2c_handle = i2c_handle;
	sensor->distance = 0.0f;

	// confirm  that the I2C lines do work with the reference registers
	uint8_t data_high, data_low;
	uint16_t data16;

	uint8_t data = 0;
	HAL_StatusTypeDef status;

	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR1_ADDR, &data);
	if (data != VL53L0X_I2C_RR1_READ){
		return HAL_ERROR;
	}

	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR2_ADDR, &data);
	if (data != VL53L0X_I2C_RR2_READ){
		return HAL_ERROR;
	}

	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR3_ADDR, &data);
	if (data != VL53L0X_I2C_RR3_READ){
		return HAL_ERROR;
	}

//	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR4_ADDR, &data_high);
//	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR4_ADDR+1, &data_low);
//	data16 = (data_high << 8) | data_low;
//	if (data16 != VL53L0X_I2C_RR4_READ){
//		return HAL_ERROR;
//	}

	// big-endian 16bit read

//	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR5_ADDR, &data_high);
//	status = VL53L0X_ReadRegister(sensor, VL53L0X_I2C_RR5_ADDR+1, &data_low);
//	data16 = (data_high << 8) | data_low;
//	if (data16 != VL53L0X_I2C_RR5_READ){
//		return HAL_ERROR;
//	}




	// DATA INITIALIZATION
	stop_variable = 0;

	// set 2v8 voltage mode (I2C baseline)
	uint8_t vhv_config_i2c = 0;
	status = VL53L0X_ReadRegister(sensor, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_i2c);
	// set last bit = 1
	vhv_config_i2c |= 0x01; // this is because we are writing to register now
	status = VL53L0X_WriteRegister(sensor, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_i2c);

	// set i2c speed
	uint8_t temp = 0x00;
	status = VL53L0X_WriteRegister(sensor, 0x88, &temp);



	temp = 0x01;
	status = VL53L0X_WriteRegister(sensor, 0x80, &temp);
	status = VL53L0X_WriteRegister(sensor, 0xFF, &temp);
	temp = 0x00;
	status = VL53L0X_WriteRegister(sensor, 0x00, &temp);
	status = VL53L0X_ReadRegister(sensor, 0x91, &stop_variable);
	temp = 0x01;
	status = VL53L0X_WriteRegister(sensor, 0x00, &temp);
	temp = 0x00;
	status = VL53L0X_WriteRegister(sensor, 0xFF, &temp);
	status = VL53L0X_WriteRegister(sensor, 0x80, &temp);

	// STATIC INITIALIZATION: default tuning settings

	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x00, 0x00);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x09, 0x00);
	write_sensor_reg(sensor, 0x10, 0x00);
	write_sensor_reg(sensor, 0x11, 0x00);
	write_sensor_reg(sensor, 0x24, 0x01);
	write_sensor_reg(sensor, 0x25, 0xFF);
	write_sensor_reg(sensor, 0x75, 0x00);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x4E, 0x2C);
	write_sensor_reg(sensor, 0x48, 0x00);
	write_sensor_reg(sensor, 0x30, 0x20);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x30, 0x09);
	write_sensor_reg(sensor, 0x54, 0x00);
	write_sensor_reg(sensor, 0x31, 0x04);
	write_sensor_reg(sensor, 0x32, 0x03);
	write_sensor_reg(sensor, 0x40, 0x83);
	write_sensor_reg(sensor, 0x46, 0x25);
	write_sensor_reg(sensor, 0x60, 0x00);
	write_sensor_reg(sensor, 0x27, 0x00);
	write_sensor_reg(sensor, 0x50, 0x06);
	write_sensor_reg(sensor, 0x51, 0x00);
	write_sensor_reg(sensor, 0x52, 0x96);
	write_sensor_reg(sensor, 0x56, 0x08);
	write_sensor_reg(sensor, 0x57, 0x30);
	write_sensor_reg(sensor, 0x61, 0x00);
	write_sensor_reg(sensor, 0x62, 0x00);
	write_sensor_reg(sensor, 0x64, 0x00);
	write_sensor_reg(sensor, 0x65, 0x00);
	write_sensor_reg(sensor, 0x66, 0xA0);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x22, 0x32);
	write_sensor_reg(sensor, 0x47, 0x14);
	write_sensor_reg(sensor, 0x49, 0xFF);
	write_sensor_reg(sensor, 0x4A, 0x00);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x7A, 0x0A);
	write_sensor_reg(sensor, 0x7B, 0x00);
	write_sensor_reg(sensor, 0x78, 0x21);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x23, 0x34);
	write_sensor_reg(sensor, 0x42, 0x00);
	write_sensor_reg(sensor, 0x44, 0xFF);
	write_sensor_reg(sensor, 0x45, 0x26);
	write_sensor_reg(sensor, 0x46, 0x05);
	write_sensor_reg(sensor, 0x40, 0x40);
	write_sensor_reg(sensor, 0x0E, 0x06);
	write_sensor_reg(sensor, 0x20, 0x1A);
	write_sensor_reg(sensor, 0x43, 0x40);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x34, 0x03);
	write_sensor_reg(sensor, 0x35, 0x44);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x31, 0x04);
	write_sensor_reg(sensor, 0x4B, 0x09);
	write_sensor_reg(sensor, 0x4C, 0x05);
	write_sensor_reg(sensor, 0x4D, 0x04);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x44, 0x00);
	write_sensor_reg(sensor, 0x45, 0x20);
	write_sensor_reg(sensor, 0x47, 0x08);
	write_sensor_reg(sensor, 0x48, 0x28);
	write_sensor_reg(sensor, 0x67, 0x00);
	write_sensor_reg(sensor, 0x70, 0x04);
	write_sensor_reg(sensor, 0x71, 0x01);
	write_sensor_reg(sensor, 0x72, 0xFE);
	write_sensor_reg(sensor, 0x76, 0x00);
	write_sensor_reg(sensor, 0x77, 0x00);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x0D, 0x01);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x80, 0x01);
	write_sensor_reg(sensor, 0x01, 0xF8);
	write_sensor_reg(sensor, 0xFF, 0x01);
	write_sensor_reg(sensor, 0x8E, 0x01);
	write_sensor_reg(sensor, 0x00, 0x01);
	write_sensor_reg(sensor, 0xFF, 0x00);
	write_sensor_reg(sensor, 0x80, 0x00);

	// now configure interrupts

	write_sensor_reg(sensor, REG_SYSTEM_INTERRUPT_CONFIG_GPIO,0x04);

	uint8_t gpio_hv_mux_active_high = 0;
	status = VL53L0X_ReadRegister(sensor, REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high);
	gpio_hv_mux_active_high &= ~0x10;
	status = VL53L0X_WriteRegister(sensor, REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high);
	write_sensor_reg(sensor, REG_SYSTEM_INTERRUPT_CLEAR,0x01);

	// last step of static init:
	temp = RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_FINAL_RANGE + RANGE_SEQUENCE_STEP_PRE_RANGE;
	VL53L0X_WriteRegister(sensor, REG_SYSTEM_SEQUENCE_CONFIG, &temp);


	// do calibration

	if (perform_single_ref_calibration(sensor, CALIBRATION_TYPE_VHV) != HAL_OK){
		return HAL_ERROR;
	}
	if (perform_single_ref_calibration(sensor, CALIBRATION_TYPE_PHASE) != HAL_OK){
		return HAL_ERROR;
	}

	temp = RANGE_SEQUENCE_STEP_DSS + RANGE_SEQUENCE_STEP_FINAL_RANGE + RANGE_SEQUENCE_STEP_PRE_RANGE;
	VL53L0X_WriteRegister(sensor, REG_SYSTEM_SEQUENCE_CONFIG, &temp);


	// sensor successfully initialized!!!

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

	return status;


}


// DATA ACQUISITION

HAL_StatusTypeDef VL53L0X_MeasureSingleDistance(VL53L0X* sensor){
	/* STEPS:
	 * 1. stop any ongoing range measuring
	 * 2. trigger new range measurement and wait for it to start
	 * 3. poll interrupt
	 * 4. read range
	 * 5. clear interrupt
	 */
	HAL_StatusTypeDef status;
	uint8_t data_high, data_low;
	uint16_t data16;

	uint8_t data = 0;

	// STEP 1
	write_sensor_reg(sensor,0x80,0x01);
	write_sensor_reg(sensor,0xFF,0x01);
	write_sensor_reg(sensor,0x00,0x00);
	write_sensor_reg(sensor,0x91,stop_variable);
	// STEP 2
	write_sensor_reg(sensor,0x00,0x01);
	write_sensor_reg(sensor,0xFF,0x00);
	write_sensor_reg(sensor,0x80,0x00);

	write_sensor_reg(sensor,REG_SYSRANGE_START,0x01);

	uint8_t sysrange_start = 0;

	do {
		status = VL53L0X_ReadRegister(sensor, REG_SYSRANGE_START, &sysrange_start);
	} while((sysrange_start & 0x01) && (status == HAL_OK));
	if (status != HAL_OK){
		return HAL_ERROR;
	}


	uint8_t interrupt_status = 0;
	do {
		status = VL53L0X_ReadRegister(sensor, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
	} while ((status == HAL_OK) && ((interrupt_status & 0x07) == 0x00));
	if (status != HAL_OK){
		return HAL_ERROR;
	}


	VL53L0X_ReadRegister(sensor, REG_RESULT_RANGE_STATUS+10, &data_high);
	VL53L0X_ReadRegister(sensor, REG_RESULT_RANGE_STATUS+11, &data_low);
	data16 = (data_high << 8) | data_low;


	//finally, clear interrupt mask
	data = 0x01;
	status = VL53L0X_WriteRegister(sensor, REG_SYSTEM_INTERRUPT_CLEAR, &data);
	if (status != HAL_OK){
		return HAL_ERROR;
	}

	// handling max value
	if (data >= VL53L0X_OUT_OF_RANGE){
		data16 = VL53L0X_OUT_OF_RANGE;
	}

	sensor->distance = data16;


	return HAL_OK;
}





// READING AND WRITING REGISTERS

HAL_StatusTypeDef VL53L0X_ReadRegister(VL53L0X* sensor, const uint8_t reg, uint8_t* pData) {
	//data is a pointer to the where data will be read to

	return HAL_I2C_Mem_Read(sensor->i2c_handle, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);

}



HAL_StatusTypeDef VL53L0X_ReadRegisters(VL53L0X* sensor, const uint8_t reg, uint8_t* pData, const uint8_t length) {
	//data is a pointer to the where data will be read to

	return HAL_I2C_Mem_Read(sensor->i2c_handle, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef VL53L0X_WriteRegister(VL53L0X* sensor, const uint8_t reg, uint8_t* pData){

	return HAL_I2C_Mem_Write(sensor->i2c_handle, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, HAL_MAX_DELAY);

}
