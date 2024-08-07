/*
 * data_processing.c
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#include "data_processing.h"

/*
 * INITIALISATION
 */
uint8_t data_processing_init(data_processing_t* data_processing, uint8_t* acc_raw_data_buf, uint8_t* gyro_raw_data_buf, uint8_t* serial_acc_raw_data_buf, uint8_t* serial_gyro_raw_data_buf, SemaphoreHandle_t* raw_data_buffer_mutex, 	SemaphoreHandle_t* serial_raw_data_buffer_mutex, uint8_t* raw_data_buffer_ready_flag, uint8_t* serial_raw_data_buffer_ready_flag) {

	/* Set struct parameters */
	data_processing->acc_raw_data_buf = acc_raw_data_buf;
	data_processing->gyro_raw_data_buf = gyro_raw_data_buf;
	data_processing->serial_acc_raw_data_buf = serial_acc_raw_data_buf;
	data_processing->serial_gyro_raw_data_buf = serial_gyro_raw_data_buf;

	data_processing->raw_data_buffer_mutex = raw_data_buffer_mutex;
	data_processing->serial_raw_data_buffer_mutex = serial_raw_data_buffer_mutex;

	data_processing->raw_data_buffer_ready_flag = raw_data_buffer_ready_flag;
	data_processing->serial_raw_data_buffer_ready_flag = serial_raw_data_buffer_ready_flag;

	uint8_t errNum = 0;

	return errNum;
}

/*
 * TASK FUNCTION
 */
void process_data_task(void *pvParameters) {
	data_processing_t* data_processing = (data_processing_t*) pvParameters;

	uint8_t acc_raw_data[6], gyro_raw_data[6];

	for (;;) {

		/* Get new data */
		for (;;) {

			/* wait for buffer to have been read */
			if (*(data_processing->raw_data_buffer_ready_flag) == 1) {

				/* wait for mutex to be free */
				if (xSemaphoreTake(data_processing->raw_data_buffer_mutex, portMAX_DELAY) == pdTRUE) {

					read_raw_data_buf(data_processing, acc_raw_data, gyro_raw_data);
					xSemaphoreGive(data_processing->raw_data_buffer_mutex); // release mutex
					*(data_processing->raw_data_buffer_ready_flag) = 0; // reset raw data ready flag
					break; // get out of while(1) loop
				}
			}
		}

		/* process new data */
		// TODO

		/* send data to serial buf*/

		/* dont send data if mutex busy, just keep going (no infinite while loop) */
		/* wait for mutex to be free */
		if (xSemaphoreTake(data_processing->serial_raw_data_buffer_mutex, portMAX_DELAY) == pdTRUE) {

			write_serial_raw_data_buf(data_processing, acc_raw_data, gyro_raw_data);
			xSemaphoreGive(data_processing->serial_raw_data_buffer_mutex); // release mutex
			*(data_processing->serial_raw_data_buffer_ready_flag) = 1; // set serial raw data ready flag
		}
	}
}

/*
 * SHARED BUFFER FUNCTION
 */
void write_serial_raw_data_buf(data_processing_t* data_processing, uint8_t* acc_data, uint8_t* gyro_data) {
	*(data_processing->serial_acc_raw_data_buf) = *acc_data;
	*(data_processing->serial_gyro_raw_data_buf) = *gyro_data;
}

void read_raw_data_buf(data_processing_t* data_processing, uint8_t* acc_data, uint8_t* gyro_data) {
	*acc_data = *(data_processing->acc_raw_data_buf);
	*gyro_data = *(data_processing->gyro_raw_data_buf);
}
