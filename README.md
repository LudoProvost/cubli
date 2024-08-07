# System Overview
Using FreeRTOS, I subdivided the system into 3 tasks: 
1. Retrieve data (sensor_interface.c)
2. Process data (data_processing.c)
3. Send data (serial_interface.c)

The sensor interface is designed to retrieve pre-determined data from the MPU6050 and the ADXL345 IC's.

The serial interface can send the data through UART to your computer. the 2 python scripts provided can be used to graph the data in real-time using matplotlib.
