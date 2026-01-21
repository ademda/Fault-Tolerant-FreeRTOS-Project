/*
 * tasks.c
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

#include "app_tasks.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdlib.h>
#include <string.h>
#include "mpu6050.h"
#include "main.h"
//COMM PROTOCOLS EXTERNS
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

//SENSORS DATA EXTERNS
extern uint16_t lum_value;
extern MPU6050_t imu;
extern MPU6050_queue_item_t imu_queue_item;
extern uint8_t *uart_rx_buffer;
//INTERTASKS COMMUNICATION EXTERNS
extern QueueHandle_t adc_sensor_queue;
extern QueueHandle_t i2c_sensor_queue;
extern QueueHandle_t uart_receiver_queue;
extern QueueHandle_t consumer_data_info_queue;

void I2CSensorTaskHandler(void *pvParameters ){
	for (;;){
		MPU6050_Read_Accel_DMA(&imu);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		MPU6050_Read_Accel_DMA_Complete(&imu);

		MPU6050_Read_Gyro_DMA(&imu);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		MPU6050_Read_Gyro_DMA_Complete(&imu);
		imu_queue_item.pitch = imu.pitch;
		imu_queue_item.roll = imu.roll;
		if (xQueueSend(i2c_sensor_queue, (void *)&imu_queue_item, 10) != pdPASS){
			Error_Handler();
		}
	//vTaskDelayUntil()
	}
	vTaskDelete(NULL);
}

void UARTReceiverTaskHandler(void *pvParameters ){
	for (;;){
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			xQueueSend(uart_receiver_queue, (void *)&uart_rx_buffer, 10);
		}
		else {
			Error_Handler();
		}
	}
	vTaskDelete(NULL);
}

void ADCSensorTaskHandler(void *pvParameters ){
	for (;;){
		HAL_ADC_Start_DMA(&hadc1, (void *)&lum_value, sizeof(lum_value));
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			xQueueSend(adc_sensor_queue, &lum_value, 10);
		}
		else {
			Error_Handler();
		}
	}
	vTaskDelete(NULL);
}
void ConsumerTaskHandler(void *pvParameters){
	for (;;){

	}
	vTaskDelete(NULL);
}
void InterfaceTaskHandler(void *pvParameters){
	for (;;){

	}
	vTaskDelete(NULL);
}
void MonitorTaskHandler(void *pvParameters ){
	for (;;){

	}
	vTaskDelete(NULL);
}
void FlowControlTaskHandler(void *pvParameters ){
	for (;;){

	}
	vTaskDelete(NULL);
}
void FaultTaskHandler(void *pvParameters ){
	for (;;){

	}
	vTaskDelete(NULL);
}
