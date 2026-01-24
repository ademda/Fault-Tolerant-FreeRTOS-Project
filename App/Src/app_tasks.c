/*
 * tasks.c
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */
#include "main.h"
#include "app_tasks.h"
#include "stm32f4xx_hal.h"
#include "uart_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdlib.h>
#include <string.h>
#include "mpu6050.h"

#define I2C_SENSOR_TASK_PERIOD_MS 	pdMS_TO_TICKS(10)
#define ADC_SENSOR_TASK_PERIOD_MS	pdMS_TO_TICKS(20) // minimus 11ms period because conversion takes =10ms
#define CONSUMER_TASK_PERIOD_MS		pdMS_TO_TICKS(10)
#define INTERFACE_TASK_PERIOD_MS	pdMS_TO_TICKS(10)
#define FLOW_CONTROL_TASK_PERIOD_MS	pdMS_TO_TICKS(50)
#define MONITOR_TASK_PERIOD_MS		pdMS_TO_TICKS(100)
#define FAULT_TASK_PERIOD

//COMM PROTOCOLS EXTERNS
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

//SENSORS DATA EXTERNS
extern uint16_t lum_value;
extern MPU6050_t imu;
extern MPU6050_queue_item_t imu_queue_item;
extern uint8_t uart_rx_buffer[UART_FRAME_SIZE];
extern char *uart_tx_buffer;

//consumer task internal variales
uint16_t con_lum_value;
uint8_t con_uart_rx_buffer[UART_FRAME_SIZE];
MPU6050_queue_item_t con_imu_queue_item;

//double buffer init
frame_t frame1;
frame_t frame2;

frame_t* write_buf = &frame1;
frame_t* read_buf = &frame2;

//INTERTASKS COMMUNICATION EXTERNS
extern QueueHandle_t adc_sensor_queue;
extern QueueHandle_t i2c_sensor_queue;
extern QueueHandle_t uart_receiver_queue;
extern QueueHandle_t consumer_data_info_queue;

// PC9: adc task CH5
// PC8: I2C task CH8
// PB8: UART task CH6
// PC6: Consumer task CH7
// PB9: Interface task
// PC5; Montioring task

void I2CSensorTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();;
	for (;;){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		MPU6050_Read_IMU_DMA(&imu);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		MPU6050_Read_IMU_DMA_Complete(&imu);
		imu_queue_item.pitch = imu.pitch;
		imu_queue_item.roll = imu.roll;
		if (xQueueSend(i2c_sensor_queue, (void *)&imu_queue_item, 10) != pdPASS){
			Error_Handler();
		}
		vTaskDelayUntil(&xLastWakeTime, I2C_SENSOR_TASK_PERIOD_MS);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}

void UARTReceiverTaskHandler(void *pvParameters ){
	HAL_UART_Receive_IT(&huart1, uart_rx_buffer, UART_FRAME_SIZE);
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			//HAL_UART_Receive_IT(&huart1, uart_rx_buffer, UART_FRAME_SIZE);
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 100);
			xQueueSend(uart_receiver_queue, (void *)&uart_rx_buffer, 10);
		}
		else {
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}

void ADCSensorTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_ADC_Start_DMA(&hadc1, (void *)&lum_value, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			xQueueSend(adc_sensor_queue, (void *)&lum_value, 10);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		else {
			Error_Handler();
		}
		vTaskDelayUntil(&xLastWakeTime, ADC_SENSOR_TASK_PERIOD_MS);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}
void ConsumerTaskHandler(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		xQueueReceive(adc_sensor_queue,&con_lum_value, 1);
		xQueueReceive(uart_receiver_queue,&con_uart_rx_buffer, 5);
		xQueueReceive(i2c_sensor_queue,&con_imu_queue_item, 1);
		write_buf->data = con_lum_value;
		write_buf->length = 2;

		frame_t* tmp = write_buf;

		write_buf = read_buf;
		read_buf = tmp;

		xTaskNotifyGive(InterfaceTask);
		vTaskDelayUntil(&xLastWakeTime, CONSUMER_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
void InterfaceTaskHandler(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		ulTaskNotifyTake(pdTrue,0);
		HAL_UART_Transmit(&huart2, read_buf->data, read_buf->length, 10); // will change this to dma/it
		//vTaskDelayUntil(&xLastWakeTime, INTERFACE_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
void MonitorTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, MONITOR_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
void FlowControlTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		vTaskDelayUntil(&xLastWakeTime, FLOW_CONTROL_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
void FaultTaskHandler(void *pvParameters ){
	for (;;){
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) != pdPASS){
			Error_Handler();
		}
	}
	vTaskDelete(NULL);
}
