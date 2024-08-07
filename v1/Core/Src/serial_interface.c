/*
 * serial_interface.c
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#include "serial_interface.h"

/*
 * INITIALISATION
 */
uint8_t serial_interface_init(serial_interface_t* serial_interface, UART_HandleTypeDef* huart, uint8_t* serial_acc_raw_data_buf, uint8_t* serial_gyro_raw_data_buf, SemaphoreHandle_t* serial_raw_data_buffer_mutex, uint8_t* serial_raw_data_buffer_ready_flag) {

	/* set struct parameters */
	serial_interface->huart = huart;

	serial_interface->serial_acc_raw_data_buf = serial_acc_raw_data_buf;
	serial_interface->serial_gyro_raw_data_buf = serial_gyro_raw_data_buf;

	serial_interface->serial_raw_data_buffer_mutex = serial_raw_data_buffer_mutex;

	serial_interface->serial_raw_data_buffer_ready_flag = serial_raw_data_buffer_ready_flag;


	uint8_t errNum = 0;

	return errNum;
}


/*
 * TASK FUNCTION
 */
void uart_interface_task(void *pvParameters) {
	serial_interface_t* serial_interface = (serial_interface_t*) pvParameters;

	uint8_t acc_raw_data[6], gyro_raw_data[6];

	for (;;) {

		if (*(serial_interface->serial_raw_data_buffer_ready_flag) == 1) {

			read_serial_raw_data_buf(serial_interface, acc_raw_data, gyro_raw_data);
			*(serial_interface->serial_raw_data_buffer_ready_flag) = 0; // reset serial raw data ready flag


			/* send data through UART */
			uint8_t serial_data[12];

			HAL_UART_Transmit(serial_interface->huart, (uint8_t *) gyro_raw_data, sizeof(gyro_raw_data), HAL_MAX_DELAY);
//			HAL_UART_Transmit(serial_interface->huart, (uint8_t *) acc_raw_data, sizeof(acc_raw_data), HAL_MAX_DELAY);
		}
		vTaskDelay(100);
	}

}

/*
 * SHARED BUFFER FUNCTION
 */
void read_serial_raw_data_buf(serial_interface_t* serial_interface, uint8_t* acc_data, uint8_t* gyro_data) {
	for (int i = 0; i<6;i++) {
		acc_data[i] = serial_interface->serial_acc_raw_data_buf[i];
		gyro_data[i] = serial_interface->serial_gyro_raw_data_buf[i];
	}
}
