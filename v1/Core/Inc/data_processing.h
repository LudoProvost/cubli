/*
 * data_processing.h
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#ifndef INC_DATA_PROCESSING_H_
#define INC_DATA_PROCESSING_H_

#include "cmsis_os.h"
#include "semphr.h"

typedef struct {
	uint8_t* acc_raw_data_buf;
	uint8_t* gyro_raw_data_buf;
	uint8_t* serial_acc_raw_data_buf;
	uint8_t* serial_gyro_raw_data_buf;

	SemaphoreHandle_t* raw_data_buffer_mutex;
	SemaphoreHandle_t* serial_raw_data_buffer_mutex;

	uint8_t* raw_data_buffer_ready_flag;
	uint8_t* serial_raw_data_buffer_ready_flag;
} data_processing_t;

/*
 * INITIALISATION
 */
uint8_t data_processing_init(data_processing_t* data_processing, uint8_t* acc_raw_data_buf, uint8_t* gyro_raw_data_buf, uint8_t* serial_acc_raw_data_buf, uint8_t* serial_gyro_raw_data_buf, SemaphoreHandle_t* raw_data_buffer_mutex, SemaphoreHandle_t* serial_raw_data_buffer_mutex, uint8_t* raw_data_buffer_ready_flag, uint8_t* serial_raw_data_buffer_ready_flag); // TODO add PWM signals buffer

/*
 * TASK FUNCTION
 */
void process_data_task(void *pvParameters);

/*
 * SHARED BUFFER FUNCTION
 */
void write_serial_raw_data_buf(data_processing_t* data_processing, uint8_t* acc_data, uint8_t* gyro_data);
void read_raw_data_buf(data_processing_t* data_processing, uint8_t* acc_data, uint8_t* gyro_data);


#endif /* INC_DATA_PROCESSING_H_ */
