/*
 * SensorInterface.c
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#include "sensor_interface.h"

/*
 * INITIALISATION
 */
uint8_t sensor_interface_init(sensor_interface_t* sensor_interface, SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c, uint8_t* acc_raw_data_buf, uint8_t* gyro_raw_data_buf, SemaphoreHandle_t* raw_data_buffer_mutex, uint8_t* acc_data_ready_int, uint8_t* gyro_data_ready_int, uint8_t* raw_data_buffer_ready_flag, ADXL345* adxl345, MPU6050* mpu6050) {

	/* Set struct parameters */
	sensor_interface->acc_raw_data_buf = acc_raw_data_buf;
	sensor_interface->gyro_raw_data_buf = gyro_raw_data_buf;

	sensor_interface->acc_data_ready_int = acc_data_ready_int;
	sensor_interface->gyro_data_ready_int = gyro_data_ready_int;
	sensor_interface->raw_data_buffer_ready_flag = raw_data_buffer_ready_flag;

	sensor_interface->raw_data_buffer_mutex = raw_data_buffer_mutex;
	sensor_interface->sampling_mode = InterruptBased;					// Setting TO CHANGE

	/* init flag */
	uint8_t flags = 0x00;

	sensor_interface->flags = flags;

	uint8_t errNum = 0;

	/* Create MPU6050 device and ADXL345 device */
	uint8_t errNumDev = 0;
//	ADXL345 adxl345;
//	MPU6050 mpu6050;

	/* ADXL345 */
	sensor_interface->adxl345 = adxl345;

	errNumDev = ADXL345_Initialise(sensor_interface->adxl345, hspi);
	errNum += errNumDev;

	/* MPU6050 */
	sensor_interface->mpu6050 = mpu6050;

	errNumDev = MPU6050_Initialise(sensor_interface->mpu6050, hi2c);
	errNum += errNumDev;

	return errNum;
}

/*
 * TASK FUNCTION
 */
void sensor_interface_task(void *pvParameters) {

	sensor_interface_t* sensor_interface = (sensor_interface_t*) pvParameters;

	uint8_t acc_raw_data[6];
	uint8_t gyro_raw_data[6];

	for (;;) {
		/* reset data retrieved flags */
		sensor_interface->flags = 0x00;	//GYRO_ACC_DATA_RETRIEVED_FLAG_MASK & 0xFF

		switch (sensor_interface->sampling_mode) {

			case InterruptBased:
				while ((sensor_interface->flags & GYRO_ACC_DATA_RETRIEVED_FLAG_MASK) != GYRO_ACC_DATA_RETRIEVED_FLAG_MASK) {
					if ((*(sensor_interface->acc_data_ready_int) == 1) && ((sensor_interface->flags & ACC_DATA_RETRIEVED_FLAG_MASK) == 0)) {

						/* Reset int flag */
						*(sensor_interface->acc_data_ready_int) = 0;

						/* read acc */
						ADXL345_ReadRawAcceleration(sensor_interface->adxl345, acc_raw_data);

						/* set acc data retrieved flag */
						sensor_interface->flags = sensor_interface->flags | (ACC_DATA_RETRIEVED_FLAG_MASK & 0xFF);
					}
					if ((*(sensor_interface->gyro_data_ready_int) == 1) && ((sensor_interface->flags & GYRO_DATA_RETRIEVED_FLAG_MASK) == 0)) {

						/* Reset int flag */
						*(sensor_interface->gyro_data_ready_int) = 0;

						/* read gyro */
						MPU6050_ReadRawGyroscope(sensor_interface->mpu6050, gyro_raw_data);

						/* set gyro data retrieved flag */
						sensor_interface->flags = sensor_interface->flags | (GYRO_DATA_RETRIEVED_FLAG_MASK & 0xFF);
					}
					vTaskDelay(10);
				}
				break;

			case FrequencyBased:
				ADXL345_ReadRawAcceleration(sensor_interface->adxl345, acc_raw_data);
				MPU6050_ReadRawGyroscope(sensor_interface->mpu6050, gyro_raw_data);
				break;

			default:
				ADXL345_ReadRawAcceleration(sensor_interface->adxl345, acc_raw_data);
				MPU6050_ReadRawGyroscope(sensor_interface->mpu6050, gyro_raw_data);
				break;
		}
//		vTaskDelay(100);
		/* At this point, new acc & gyro data have been gathered, prepare to write to raw data buffer */

		/* wait for mutex to be free & for data to have been read by data processing task*/
		while (1) {

			/* wait for buffer to have been read */
			if (*sensor_interface->raw_data_buffer_ready_flag == 0) {

				write_raw_data_buf(sensor_interface, acc_raw_data, gyro_raw_data);
				*(sensor_interface->raw_data_buffer_ready_flag) = 1; // set raw data ready flag
				break; // get out of while(1) loop
			} else {
				vTaskDelay(100);
			}
		}
//
//		// TODO change this so its cleaner...
//		if (sensor_interface->sampling_mode == FrequencyBased) {
//			vTaskDelay(500);											//HARDCODED SAMPLING PERIOD
//		}
	}
}

/*
 * SHARED BUFFER FUNCTION
 */
void write_raw_data_buf(sensor_interface_t* sensor_interface, uint8_t* acc_data, uint8_t* gyro_data) {
	for (int i = 0; i<6;i++) {
		sensor_interface->acc_raw_data_buf[i] = acc_data[i];
		sensor_interface->gyro_raw_data_buf[i] = gyro_data[i];
	}
}
