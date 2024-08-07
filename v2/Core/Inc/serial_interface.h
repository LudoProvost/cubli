/*
 * serial_interface.h
 *
 *  Created on: May 7, 2024
 *      Author: Ludovic Provost
 */

#ifndef INC_SERIAL_INTERFACE_H_
#define INC_SERIAL_INTERFACE_H_

#include "stm32f3xx_hal.h"

#include "cmsis_os.h"
#include "semphr.h"

typedef struct {
	UART_HandleTypeDef* huart;
	uint8_t* serial_acc_raw_data_buf;
	uint8_t* serial_gyro_raw_data_buf;
	SemaphoreHandle_t* serial_raw_data_buffer_mutex;
	uint8_t* serial_raw_data_buffer_ready_flag;
} serial_interface_t;


/*
 * INITIALISATION
 */
uint8_t serial_interface_init(serial_interface_t* serial_interface, UART_HandleTypeDef* huart, uint8_t* serial_acc_raw_data_buf, uint8_t* serial_gyro_raw_data_buf, SemaphoreHandle_t* serial_raw_data_buffer_mutex, uint8_t* serial_raw_data_buffer_ready_flag); // TODO add PWM signals buffer


/*
 * TASK FUNCTION
 */
void uart_interface_task(void *pvParameters);

/*
 * SHARED BUFFER FUNCTION
 */
void read_serial_raw_data_buf(serial_interface_t* serial_interface, uint8_t* acc_data, uint8_t* gyro_data);

#endif /* INC_SERIAL_INTERFACE_H_ */
