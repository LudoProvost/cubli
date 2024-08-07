/*
 * mpu6050.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Ludovic Provost
 */

#include "mpu6050.h"
#include "stm32f3xx_hal.h"
#include "stdio.h"		// needed for debug status


/*
 * INITIALIZATION
 */
uint8_t MPU6050_Initialise(MPU6050* dev, I2C_HandleTypeDef* hi2c) {

	/* INPUT FOR DESIRED RANGE*/
	/* Also used to initialize dev->sensitivity inside accel config and gyro config */
	enum AccelRange A_range = range_4g;
	enum GyroRange G_range = range_250deg;

	/* init dev param */
	dev->hi2c = hi2c;

	dev->acc[0] = 0;
	dev->acc[1] = 0;
	dev->acc[2] = 0;

	dev->gyro[0] = 0;
	dev->gyro[1] = 0;
	dev->gyro[2] = 0;

	/* device check */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t who_am_i;
	status = MPU6050_ReadRegister(dev, MPU6050_REG_WHO_AM_I, &who_am_i);
	errNum += (status != HAL_OK);

	if ((who_am_i << 1) != MPU6050_I2C_ADDR) {
		return 255;
	}

	/* Accel & Gyro config */
	status = MPU6050_AccelConfig(dev, A_range);
	errNum += (status != HAL_OK);

	status = MPU6050_GyroConfig(dev, G_range);
	errNum += (status != HAL_OK);

	/* interrupt config */
	status = MPU6050_InterruptConfig(dev, DATA_RDY);
	errNum += (status != HAL_OK);

	/* Take device out of sleep mode (p.40 of reg. map) */
	uint8_t reg_data = 0x00;
	status = MPU6050_WriteRegister(dev, MPU6050_REG_PWR_MGMT_1, &reg_data, 1);
	errNum += (status != HAL_OK);

	return errNum;
}

HAL_StatusTypeDef MPU6050_AccelConfig(MPU6050* dev, enum AccelRange range) {
	HAL_StatusTypeDef status;
	uint8_t accel_config_reg;

	/* sets accel_config_reg to right value & initializes LSB sensitivity */
	switch (range) {
		case range_2g:
			dev->A_LSB_sensitivity = 16384.0;
			accel_config_reg = 0x00;
			break;
		case range_4g:
			dev->A_LSB_sensitivity = 8192.0;
			accel_config_reg = 0x08;
			break;
		case range_8g:
			dev->A_LSB_sensitivity = 4096.0;
			accel_config_reg = 0x10;
			break;
		case range_16g:
			dev->A_LSB_sensitivity = 2048.0;
			accel_config_reg = 0x18;
			break;
		default: // default is +-2g
			dev->A_LSB_sensitivity = 16384.0;
			accel_config_reg = 0x00;
	}

	status = MPU6050_WriteRegister(dev, MPU6050_REG_ACCEL_CONFIG, &accel_config_reg, 1);
	return status;
}

HAL_StatusTypeDef MPU6050_GyroConfig(MPU6050* dev, enum GyroRange range) {
	HAL_StatusTypeDef status;
	uint8_t gyro_config_reg;

	/* sets gyro_config_reg to right value & initializes LSB sensitivity */
	switch (range) {
		case range_250deg:
			dev->G_LSB_sensitivity = 131.0;
			gyro_config_reg = 0x00;
			break;
		case range_500deg:
			dev->G_LSB_sensitivity = 65.5;
			gyro_config_reg = 0x08;
			break;
		case range_1000deg:
			dev->G_LSB_sensitivity = 32.8;
			gyro_config_reg = 0x10;
			break;
		case range_2000deg:
			dev->G_LSB_sensitivity = 16.4;
			gyro_config_reg = 0x18;
			break;
		default: // default is 250deg
			dev->G_LSB_sensitivity = 131.0;
			gyro_config_reg = 0x00;
	}

	status = MPU6050_WriteRegister(dev, MPU6050_REG_GYRO_CONFIG, &gyro_config_reg, 1);
	return status;
}

HAL_StatusTypeDef MPU6050_InterruptConfig(MPU6050* dev, enum InterruptSource source) {
	HAL_StatusTypeDef status;
	uint8_t int_enable_reg;

	switch (source) {
		case DATA_RDY:
			int_enable_reg = 0x01;
			break;
		case MST:
			int_enable_reg = 0x08;
			break;
		case OFLOW:
			int_enable_reg = 0x10;
			break;
		default: // default is no interrupt
			int_enable_reg = 0x00;
	}

	status = MPU6050_WriteRegister(dev, MPU6050_REG_INT_ENABLE, &int_enable_reg, 1);

	/* sets INT_PIN_CFG register -> LATCH_INT_EN bit to 1 & INT_RD_CLEAR bit to 1 (p.26 of register map) */
	uint8_t reg = 0x30;
	status = MPU6050_WriteRegister(dev, MPU6050_REG_INT_PIN_CFG, &reg, 1);

	return status;
}

/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef MPU6050_ReadAcceleration(MPU6050* dev) {
	HAL_StatusTypeDef status;

	/* get raw register data */
	uint8_t reg_data[6];

	status = MPU6050_ReadRegisters(dev, MPU6050_REG_ACCEL_XOUT_H, reg_data, sizeof(reg_data));

	/* combine registers */
	uint16_t raw_data[3];

	raw_data[0] = ((uint16_t) reg_data[0] << 8) | reg_data[1];	// X out
	raw_data[1] = ((uint16_t) reg_data[2] << 8) | reg_data[3];	// Y out
	raw_data[2] = ((uint16_t) reg_data[4] << 8) | reg_data[5];	// Z out

	/* convert to signed data */
	int16_t signed_data[3];
	signed_data[0] = (int16_t) raw_data[0];
	signed_data[1] = (int16_t) raw_data[1];
	signed_data[2] = (int16_t) raw_data[2];

	/* store in dev struct */
	dev->acc[0] = signed_data[0] / dev->A_LSB_sensitivity;
	dev->acc[1] = signed_data[1] / dev->A_LSB_sensitivity;
	dev->acc[2] = signed_data[2] / dev->A_LSB_sensitivity;

	return status;
}

HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050* dev) {
	HAL_StatusTypeDef status;

	/* get raw register data */
	uint8_t reg_data[6];

	status = MPU6050_ReadRegisters(dev, MPU6050_REG_GYRO_XOUT_H, reg_data, sizeof(reg_data));

	/* combine registers */
	uint16_t raw_data[3];

	raw_data[0] = ((uint16_t) reg_data[0] << 8) | reg_data[1];	// X out
	raw_data[1] = ((uint16_t) reg_data[2] << 8) | reg_data[3];	// Y out
	raw_data[2] = ((uint16_t) reg_data[4] << 8) | reg_data[5];	// Z out

	/* convert to signed data */
	int16_t signed_data[3];
	signed_data[0] = (int16_t) raw_data[0];
	signed_data[1] = (int16_t) raw_data[1];
	signed_data[2] = (int16_t) raw_data[2];

	/* store in dev struct */

	dev->gyro[0] = signed_data[0] / dev->G_LSB_sensitivity;
	dev->gyro[1] = signed_data[1] / dev->G_LSB_sensitivity;
	dev->gyro[2] = signed_data[2] / dev->G_LSB_sensitivity;

	return status;
}

HAL_StatusTypeDef MPU6050_ReadRawAcceleration(MPU6050* dev, uint8_t* buf) {
	HAL_StatusTypeDef status;

	status = MPU6050_ReadRegisters(dev, MPU6050_REG_ACCEL_XOUT_H, buf, 6);

	return status;
}

HAL_StatusTypeDef MPU6050_ReadRawGyroscope(MPU6050* dev, uint8_t* buf) {
	HAL_StatusTypeDef status;

	status = MPU6050_ReadRegisters(dev, MPU6050_REG_GYRO_XOUT_H, buf, 6);

	return status;
}

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* dev, uint8_t reg, uint8_t* buf, uint8_t size) {
	return HAL_I2C_Mem_Read(dev->hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, size, 100);
}

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* dev, uint8_t reg, uint8_t* buf) {
	return HAL_I2C_Mem_Read(dev->hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* dev, uint8_t reg, uint8_t* buf, uint8_t size) {
	return HAL_I2C_Mem_Write(dev->hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, size, 100);
}

/*
 * DEBUG
 */
uint8_t MPU6050_GetStatus(MPU6050* dev, UART_HandleTypeDef* huart) {
	return 1;
}

void MPU6050_DebugGyro(MPU6050* dev, UART_HandleTypeDef* huart) {
	char buf[30];
	float x,y,z;

	x = dev->gyro[0];
	y = dev->gyro[1];
	z = dev->gyro[2];

	sprintf(buf, " %+.1f %+.1f %+.1f\n", x, y, z);
	// Send the formatted string via UART
	HAL_UART_Transmit(huart, (uint8_t *) buf, sizeof(buf), HAL_MAX_DELAY);
}

void MPU6050_DebugAccel(MPU6050* dev, UART_HandleTypeDef* huart) {
	char buf[30];
	float x,y,z;

	x = dev->acc[0];
	y = dev->acc[1];
	z = dev->acc[2];

	sprintf(buf, " %+.1f %+.1f %+.1f\n", x, y, z);
	// Send the formatted string via UART
	HAL_UART_Transmit(huart, (uint8_t *) buf, sizeof(buf), HAL_MAX_DELAY);
}

void MPU6050_DebugRawGyro(MPU6050* dev, UART_HandleTypeDef* huart) {
	HAL_StatusTypeDef status;

	/* get raw register data */
	uint8_t reg_data[6];

	status = MPU6050_ReadRegisters(dev, MPU6050_REG_GYRO_XOUT_H, reg_data, sizeof(reg_data));

	if (status != HAL_OK) {
		status = 1;
	}

	if (sizeof(reg_data) < 6) {
		status = 1;
	}

	HAL_UART_Transmit(huart, (uint8_t *) reg_data, sizeof(reg_data), HAL_MAX_DELAY);
}
