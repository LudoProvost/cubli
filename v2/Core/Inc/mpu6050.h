/*
 * mpu6050.h
 *
 *  Created on: Apr 23, 2024
 *      Author: Ludovic Provost
 *
 *  Datasheet:		https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  Register map:	https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f3xx_hal.h"


/*
 * DEFINES
 */
#define MPU6050_I2C_ADDR 	(0x68 << 1)

/*
 * REGISTERS (p.6-7-8 of register map)
 */
#define MPU6050_REG_SMPLRT_DIV		0x19
#define MPU6050_REG_CONFIG			0x1A
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C
#define MPU6050_REG_INT_PIN_CFG		0x37
#define MPU6050_REG_INT_ENABLE		0x38
#define MPU6050_REG_INT_STATUS		0x3A
#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_GYRO_XOUT_H		0x43
#define MPU6050_REG_PWR_MGMT_1		0X6B
#define MPU6050_REG_PWR_MGMT_2		0X6C
#define MPU6050_REG_WHO_AM_I		0x75

/* p.15 of reg map */
enum AccelRange {
	range_2g,
	range_4g,
	range_8g,
	range_16g
};

/* p.14 of reg map */
enum GyroRange {
	range_250deg,
	range_500deg,
	range_1000deg,
	range_2000deg
};

/* p.27 of reg map */
enum InterruptSource {
	OFLOW,
	MST,
	DATA_RDY
};

/*
 * SENSOR STRUCT
 */
typedef struct {
	I2C_HandleTypeDef* hi2c;
	float acc[3];	// (X,Y,Z) in mps^2
	float gyro[3];	// (X,Y,Z) in deg/s
	float A_LSB_sensitivity;	// LSB/g
	float G_LSB_sensitivity;	// LSB/deg/s
} MPU6050;

/*
 * INITIALIZATION
 */
uint8_t MPU6050_Initialise(MPU6050* dev, I2C_HandleTypeDef* hi2c);

HAL_StatusTypeDef MPU6050_AccelConfig(MPU6050* dev, enum AccelRange range);
HAL_StatusTypeDef MPU6050_GyroConfig(MPU6050* dev, enum GyroRange range);
HAL_StatusTypeDef MPU6050_InterruptConfig(MPU6050* dev, enum InterruptSource source);

/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef MPU6050_ReadAcceleration(MPU6050* dev);
HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050* dev);
HAL_StatusTypeDef MPU6050_ReadRawAcceleration(MPU6050* dev, uint8_t* buf);
HAL_StatusTypeDef MPU6050_ReadRawGyroscope(MPU6050* dev, uint8_t* buf);

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* dev, uint8_t reg, uint8_t* buf, uint8_t size);
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* dev, uint8_t reg, uint8_t* buf);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* dev, uint8_t reg, uint8_t* buf, uint8_t size);

/*
 * DEBUG
 */
uint8_t MPU6050_GetStatus(MPU6050* dev, UART_HandleTypeDef* huart);
void MPU6050_DebugGyro(MPU6050* dev, UART_HandleTypeDef* huart);
void MPU6050_DebugAccel(MPU6050* dev, UART_HandleTypeDef* huart);

void MPU6050_DebugRawGyro(MPU6050* dev, UART_HandleTypeDef* huart);

#endif /* INC_MPU6050_H_ */
