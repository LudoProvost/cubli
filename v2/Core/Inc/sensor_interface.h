/*
 * sensorInterface.h
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#ifndef INC_SENSOR_INTERFACE_H_
#define INC_SENSOR_INTERFACE_H_

#include "ADXL345.h"
#include "MPU6050.h"

#include "cmsis_os.h"
#include "semphr.h"


/*
 *
 */
#define ACC_DATA_RETRIEVED_FLAG_MASK		0x01
#define GYRO_DATA_RETRIEVED_FLAG_MASK		0x02
#define GYRO_ACC_DATA_RETRIEVED_FLAG_MASK	0x03

enum SamplingMode{
	InterruptBased,
	FrequencyBased
};

typedef struct {
	ADXL345* adxl345;
	MPU6050* mpu6050;
	uint8_t* acc_raw_data_buf;
	uint8_t* gyro_raw_data_buf;
	uint8_t* acc_data_ready_int;
	uint8_t* gyro_data_ready_int;
	uint8_t* raw_data_buffer_ready_flag;
	SemaphoreHandle_t* raw_data_buffer_mutex;
	enum SamplingMode sampling_mode;
	uint8_t flags;	// active high. bit0 => accelerometer data retrieved, bit1 = gyroscope data retrieved
} sensor_interface_t;

/*
 * INITIALISATION
 */
uint8_t sensor_interface_init(sensor_interface_t* sensor_interface, SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c, uint8_t* acc_raw_data_buf, uint8_t* gyro_raw_data_buf, SemaphoreHandle_t* raw_data_buffer_mutex, uint8_t* acc_data_ready_int, uint8_t* gyro_data_ready_int, uint8_t* raw_data_buffer_ready_flag, ADXL345* adxl345, MPU6050* mpu6050);	//TODO add interrupt data ready flags

/*
 * TASK FUNCTION
 */
void sensor_interface_task(void *pvParameters);

/*
 * SHARED BUFFER FUNCTION
 */
void write_raw_data_buf(sensor_interface_t* sensor_interface, uint8_t* acc_data, uint8_t* gyro_data);


#endif /* INC_SENSOR_INTERFACE_H_ */
