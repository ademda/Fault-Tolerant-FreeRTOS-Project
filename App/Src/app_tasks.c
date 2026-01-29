/*
 * tasks.c
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

/* THE data arbitration policy that will be used for the arbitration between the 3 producers is RT_Lottery
 * 1st layer is : RT: Real time handler : we set a warning_line for each producer and if one producer goes
 * higher than that line the bus (UART->PC) will be granted to him...if 2 producers goes below the line
 * the one with the highest ratio will get granted
 * 2nd layer: lottery arbitration: based on a weight for each producer a number of tickets will be generated
 * for that master: at each cycle of control flow a random ticket number will be generated and the producer
 * having that ticket number will get the bus ownership
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
#include "semphr.h"

#define I2C_SENSOR_TASK_PERIOD_MS 	pdMS_TO_TICKS(10)
#define ADC_SENSOR_TASK_PERIOD_MS	pdMS_TO_TICKS(20) // minimus 11ms period because conversion takes =10ms
#define CONSUMER_TASK_PERIOD_MS		pdMS_TO_TICKS(10)
#define INTERFACE_TASK_PERIOD_MS	pdMS_TO_TICKS(10)
#define FLOW_CONTROL_TASK_PERIOD_MS	pdMS_TO_TICKS(25)
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
uint32_t rng_state = 2463534242u;

//6bit tickets 64
//double buffer init
frame_t frame1;
production_arbiter_t arbiter_decision = ADC_OWNERSHIP;
frame_t* write_buf = &frame1;

//SEMAPHORES and MUTEXES
extern SemaphoreHandle_t arbitration_mutex;

//INTERTASKS COMMUNICATION EXTERNS
extern QueueHandle_t adc_sensor_queue;
extern QueueHandle_t i2c_sensor_queue;
extern QueueHandle_t uart_receiver_queue;

// PC9: adc task CH5
// PC8: I2C task CH8
// PB8: UART task CH6
// PC6: Consumer task CH7
// PB9: Interface task
// PC5; Montioring task

//CH1 UART RX
//CH2 I2C CLK
//CH3 I2C DATA
//CH4 UART TX
static inline uint32_t xorshift32_star(void)
{
    uint32_t x = rng_state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng_state = x;
    x = x * 0x9E3779BBu;
    return  (x * 0x9E3779BBu) >> 26;
}

void I2CSensorTaskHandler(void *pvParameters ){
	  MPU6050_Init(&imu);
	TickType_t xLastWakeTime = xTaskGetTickCount();

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
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, I2C_SENSOR_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void UARTReceiverTaskHandler(void *pvParameters ){
	HAL_UART_Receive_IT(&huart1, uart_rx_buffer, UART_FRAME_SIZE);
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer), 100);
			xQueueSend(uart_receiver_queue, (void *)&uart_rx_buffer, 10); //choukouk lahna
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

		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS){
			xQueueSend(adc_sensor_queue, (void *)&lum_value, 10);

		}
		else {
			Error_Handler();
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, ADC_SENSOR_TASK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void ConsumerTaskHandler(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		xQueueReceive(adc_sensor_queue,&con_lum_value, 1);
		xQueueReceive(uart_receiver_queue,&con_uart_rx_buffer, 5); //choukouk lahna
		xQueueReceive(i2c_sensor_queue,&con_imu_queue_item, 1);
		xSemaphoreTake(arbitration_mutex, portMAX_DELAY);
		if (arbiter_decision == ADC_OWNERSHIP){
			write_buf->data[0] = con_lum_value & 0xFF;
			write_buf->data[1] = (con_lum_value & 0xFF00) >> 8;
			write_buf->length = 2;
		}
		else if (arbiter_decision == I2C_OWNERSHIP){
			write_buf->data[0] = con_imu_queue_item.pitch;
			write_buf->data[1] = con_imu_queue_item.roll;
			write_buf->length = 2;
		}
		else if (arbiter_decision == UART_OWNERSHIP){
			memcpy(write_buf, con_uart_rx_buffer, UART_FRAME_SIZE);
			write_buf->length = strlen((char *)con_uart_rx_buffer);
		}
		xSemaphoreGive(arbitration_mutex);
		//HAL_UART_Transmit_DMA(&huart2, (uint8_t *)write_buf->data, write_buf->length);
		HAL_UART_Transmit(&huart2, (uint8_t *)write_buf->data, write_buf->length, 1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime, CONSUMER_TASK_PERIOD_MS);
		/*if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) != pdPASS){
			Error_Handler();
		}*/
	}
	vTaskDelete(NULL);
}


void FlowControlTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i2c_sensor_queue_consumption = 0;
	uint32_t uart_receiver_queue_consumption = 0;
	uint32_t adc_sensor_queue_consumption = 0;
	uint32_t i2c_sensor_queue_ratio = 0;
	uint32_t uart_receiver_queue_ratio = 0;
	uint32_t adc_sensor_queue_ratio = 0;
	for (;;){
		i2c_sensor_queue_consumption = uxQueueMessagesWaiting(i2c_sensor_queue);
		adc_sensor_queue_consumption = uxQueueMessagesWaiting(adc_sensor_queue);
		uart_receiver_queue_consumption = uxQueueMessagesWaiting(uart_receiver_queue);
		i2c_sensor_queue_ratio = i2c_sensor_queue_consumption/ADC_QUEUE_SIZE;
		adc_sensor_queue_ratio = adc_sensor_queue_consumption/ADC_QUEUE_SIZE;
		uart_receiver_queue_ratio = uart_receiver_queue_consumption / UART_QUEUE_SIZE;
		uint16_t random_ticket = xorshift32_star();
		xSemaphoreTake(arbitration_mutex, portMAX_DELAY);
		if (adc_sensor_queue_ratio >= ADC_MAX_CONSUMPTION_RATIO){
			arbiter_decision = ADC_OWNERSHIP;
		}
		else if(uart_receiver_queue_ratio >= UART_MAX_CONSUMPTION_RATIO){
			arbiter_decision = UART_OWNERSHIP;
		}
		else if(i2c_sensor_queue_ratio >= I2C_MAX_CONSUMPTION_RATIO){
			arbiter_decision = I2C_OWNERSHIP;
		}
		else {
			if (random_ticket >= ADC_MIN_LOTTERY_TICKET && random_ticket <= ADC_MAX_LOTTERY_TICKET){
				arbiter_decision = ADC_OWNERSHIP;
			}
			else if (random_ticket >= UART_MIN_LOTTERY_TICKET && random_ticket <= UART_MAX_LOTTERY_TICKET){
				arbiter_decision = UART_OWNERSHIP;
			}
			else {
				arbiter_decision = I2C_OWNERSHIP;
			}
		}
		xSemaphoreGive(arbitration_mutex);
		vTaskDelayUntil(&xLastWakeTime, FLOW_CONTROL_TASK_PERIOD_MS);
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

void FaultTaskHandler(void *pvParameters ){
	for (;;){
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) != pdPASS){
			Error_Handler();
		}
	}
	vTaskDelete(NULL);
}
