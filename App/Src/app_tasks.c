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
#include <stdio.h>
#include "mpu6050.h"
#include "semphr.h"
#include "event_groups.h"

//#define FLOW_CONTROL_DEBUG 1
#define UART_TASK_DEBUG 1 //i don't have a lot of channels in the logic analyzer so i will use the Flow control task channel for the uart
#define FAULT_TASK_DEBUG 1

#define I2C_SENSOR_TASK_PERIOD_MS	3 //2MS is the maximum from the frame on the logic analyzer //2best //30
#define ADC_SENSOR_TASK_PERIOD_MS	2 // minimus 11ms period because conversion takes =10ms //2best //30
#define FLOW_CONTROL_TASK_PERIOD_MS	15

#define I2C_SENSOR_TASK_PERIOD_TICKS 	pdMS_TO_TICKS(I2C_SENSOR_TASK_PERIOD_MS) //2MS is the maximum from the frame on the logic analyzer //2best //30
#define ADC_SENSOR_TASK_PERIOD_TICKS	pdMS_TO_TICKS(ADC_SENSOR_TASK_PERIOD_MS) // minimus 11ms period because conversion takes =10ms //2best //30
#define FLOW_CONTROL_TASK_PERIOD_TICKS	pdMS_TO_TICKS(FLOW_CONTROL_TASK_PERIOD_MS)

#define ADC_TASK_DEADLINE			ADC_SENSOR_TASK_PERIOD_MS + 2
#define I2C_TASK_DEADLINE			I2C_SENSOR_TASK_PERIOD_MS + 5
#define FLOW_CONTROL_TASK_DEADLINE  FLOW_CONTROL_TASK_PERIOD_MS + 5


#define ADC_TASK_BIT			0
#define I2C_TASK_BIT			1
#define FLOW_CONTROL_TASK_BIT	2

#define ADC_TASK_DEADLINE_MISS_FLAG 			0x1
#define I2C_TASK_DEADLINE_MISS_FLAG 			0x2
#define FLOW_CONTROL_TASK_DEADLINE_MISS_FLAG	0x3

#define BIT_0	1 << 0 //adc bit
#define BIT_1	1 << 1 //i2c bit
#define BIT_2	1 << 2 //flow control bit
//COMM PROTOCOLS EXTERNS
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern IWDG_HandleTypeDef hiwdg;
//SENSORS DATA EXTERNS
extern uint16_t lum_value;
extern MPU6050_t imu;
extern MPU6050_queue_item_t imu_queue_item;
extern uint8_t uart_rx_buffer[UART_FRAME_SIZE];
extern uint8_t uart_rx_char;
extern char *uart_tx_buffer;

//SEMAPHORES and MUTEXES
extern SemaphoreHandle_t arbitration_mutex;
extern SemaphoreHandle_t i2c_stack_usage_mutex;
extern SemaphoreHandle_t adc_stack_usage_mutex;
extern SemaphoreHandle_t uart_stack_usage_mutex;
extern SemaphoreHandle_t consumer_stack_usage_mutex;
extern SemaphoreHandle_t flow_control_stack_usage_mutex;
extern SemaphoreHandle_t fault_handler_stack_usage_mutex;
extern SemaphoreHandle_t watchdog_stack_usage_mutex;
//EVENT GROUPS
extern EventGroupHandle_t wdog_event_group;

//INTERTASKS COMMUNICATION EXTERNS
extern QueueHandle_t adc_sensor_queue;
extern QueueHandle_t i2c_sensor_queue;
extern QueueHandle_t uart_receiver_queue;

//Fault Handler task handle
extern TaskHandle_t I2CSensorTask;
extern TaskHandle_t ADCSensorTask;
extern TaskHandle_t UARTReceiverTask;
extern TaskHandle_t ConsumerTask;
extern TaskHandle_t FlowControlTask;
extern TaskHandle_t FaultHandlerTask;
extern TaskHandle_t WatchDogTask;
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
char* overflow_msg = "stack overflowed watchdog will reset system\r\n";
//stack usage variables


typedef struct {
	EventBits_t bit;
	TickType_t firstTick;
	TickType_t lastTick;
	TickType_t deadline;
}wdog_task_t;

wdog_task_t wdog_task [] = {
	{ADC_TASK_BIT, 0, 0,ADC_TASK_DEADLINE},
	{I2C_TASK_BIT, 0, 0,I2C_TASK_DEADLINE},
	{FLOW_CONTROL_TASK_BIT, 0, 0,FLOW_CONTROL_TASK_DEADLINE}
};

// PC9: adc task CH1
// PC8: I2C task CH8
// PB8: IDLE task CH6
// PC6: Consumer task CH7
// PB9: Interface task
// PC5; FLOW control task / UART

//CH1 UART RX
//CH2 I2C CLK
//CH3 I2C DATA
//CH4 UART TX
uint32_t xorshift32_star(void)
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
	TickType_t xLastWakeTime = pdTICKS_TO_MS(xTaskGetTickCount());
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		wdog_task[1].firstTick = pdTICKS_TO_MS(xTaskGetTickCount());
		MPU6050_Read_IMU_DMA(&imu);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		MPU6050_Read_IMU_DMA_Complete(&imu);
		imu_queue_item.pitch = imu.pitch;
		imu_queue_item.roll = imu.roll;
		xQueueSend(i2c_sensor_queue, (void *)&imu_queue_item, 0);
		wdog_task[1].lastTick = pdTICKS_TO_MS(xTaskGetTickCount());
		xEventGroupSetBits(wdog_event_group, BIT_1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

		vTaskDelayUntil(&xLastWakeTime, I2C_SENSOR_TASK_PERIOD_TICKS);
	}
	vTaskDelete(NULL);
}

void UARTReceiverTaskHandler(void *pvParameters ){
	HAL_UART_Receive_IT(&huart1, uart_rx_buffer, UART_FRAME_SIZE);
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#ifndef FAULT_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
		 xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
#ifndef	FAULT_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
#endif
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		uint8_t local_buffer[UART_FRAME_SIZE] = {0};
		memcpy(local_buffer, uart_rx_buffer, UART_FRAME_SIZE);
		xQueueSend(uart_receiver_queue, local_buffer, 0); //choukouk lahna
		memset(uart_rx_buffer,0, UART_FRAME_SIZE);
		HAL_UART_Receive_IT(&huart1, uart_rx_buffer, UART_FRAME_SIZE);
#ifndef	FAULT_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
	}
	vTaskDelete(NULL);
}

void ADCSensorTaskHandler(void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		wdog_task[0].firstTick = pdTICKS_TO_MS(xTaskGetTickCount());
		HAL_ADC_Start_DMA(&hadc1, (void *)&lum_value, 1);
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		xQueueSend(adc_sensor_queue, (void *)&lum_value, 0);
		wdog_task[0].lastTick = pdTICKS_TO_MS(xTaskGetTickCount());
		xEventGroupSetBits(wdog_event_group, BIT_0);
		vTaskDelayUntil(&xLastWakeTime, ADC_SENSOR_TASK_PERIOD_TICKS);
	}
	vTaskDelete(NULL);
}

void ConsumerTaskHandler(void *pvParameters){
	char *msg ="idle\n";
	for (;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		xQueueReceive(adc_sensor_queue,&con_lum_value, 0);
		xQueueReceive(uart_receiver_queue,con_uart_rx_buffer, 0);
		xQueueReceive(i2c_sensor_queue,&con_imu_queue_item, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		xSemaphoreTake(arbitration_mutex, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		if (arbiter_decision == ADC_OWNERSHIP){
			write_buf->length = sprintf((char *)write_buf->data,"ADC: %d\r\n", con_lum_value);
			msg = "adc\r\n";
		}
		else if (arbiter_decision == I2C_OWNERSHIP){
			write_buf->length = sprintf((char *)write_buf->data,"Roll, %f Pitch: %f\r\n", con_imu_queue_item.pitch, con_imu_queue_item.roll);
			msg = "I2C\r\n";
		}
		else if (arbiter_decision == UART_OWNERSHIP){
			memset(write_buf->data,0,write_buf->length);
			memcpy(write_buf->data, con_uart_rx_buffer, UART_FRAME_SIZE);
			strcat((char *)write_buf->data,"\r\n");
			write_buf->length = strlen((char *)write_buf->data);
			msg = "uart\r\n";
		}
		else {
			msg = "aaa\r\n";
		}
		xSemaphoreGive(arbitration_mutex);
#ifndef FLOW_CONTROL_DEBUG
		HAL_UART_Transmit_IT(&huart2, write_buf->data, write_buf->length);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
#endif
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
	char *debug = "debug";
	for (;;){
#ifndef UART_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
#endif

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		wdog_task[2].firstTick = pdTICKS_TO_MS(xTaskGetTickCount());
		i2c_sensor_queue_consumption = uxQueueMessagesWaiting(i2c_sensor_queue);
		adc_sensor_queue_consumption = uxQueueMessagesWaiting(adc_sensor_queue);
		uart_receiver_queue_consumption = uxQueueMessagesWaiting(uart_receiver_queue);
		i2c_sensor_queue_ratio = i2c_sensor_queue_consumption/ADC_QUEUE_SIZE;
		adc_sensor_queue_ratio = adc_sensor_queue_consumption/ADC_QUEUE_SIZE;
		uart_receiver_queue_ratio = uart_receiver_queue_consumption / UART_QUEUE_SIZE;
		uint16_t random_ticket = xorshift32_star();
#ifndef UART_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
		xSemaphoreTake(arbitration_mutex, portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#ifndef UART_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
#endif
		if(i2c_sensor_queue_ratio >= I2C_MAX_CONSUMPTION_RATIO){
			arbiter_decision = I2C_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
			debug = "i2c_critical\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
		}
		else if (adc_sensor_queue_ratio >= ADC_MAX_CONSUMPTION_RATIO){
			arbiter_decision = ADC_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
			debug = "adc_critical\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
		}
		else if(uart_receiver_queue_ratio >= UART_MAX_CONSUMPTION_RATIO){
			arbiter_decision = UART_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
			debug = "uart_critical\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
		}
		else {
			if (random_ticket >= ADC_MIN_LOTTERY_TICKET && random_ticket <= ADC_MAX_LOTTERY_TICKET){
				arbiter_decision = ADC_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
				debug = "adc_normal\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
			}
			else if (random_ticket >= UART_MIN_LOTTERY_TICKET && random_ticket <= UART_MAX_LOTTERY_TICKET){
				arbiter_decision = UART_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
				debug = "uart_normal\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
			}
			else {
				arbiter_decision = I2C_OWNERSHIP;
#ifdef FLOW_CONTROL_DEBUG
				debug = "i2c_normal\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *)debug, strlen(debug), 1000);
#endif
			}
		}
		xSemaphoreGive(arbitration_mutex);
#ifndef UART_TASK_DEBUG
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
		wdog_task[2].lastTick = pdTICKS_TO_MS(xTaskGetTickCount());
		xEventGroupSetBits(wdog_event_group, BIT_2);
		vTaskDelayUntil(&xLastWakeTime, FLOW_CONTROL_TASK_PERIOD_TICKS);
	}
	vTaskDelete(NULL);
}

void FaultTaskHandler(void *pvParameters ){
	task_deadline_counter_t task_deadline;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	memset(&task_deadline, 0, sizeof(task_deadline));
	uint32_t ulNotificationValue = 0;
	char* log_msg;
	for (;;){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		xTaskNotifyWait(0x00,0xFFFFFFFF,&ulNotificationValue,portMAX_DELAY);
		if (ulNotificationValue & ADC_TASK_DEADLINE_MISS_FLAG ){
			task_deadline.adc++;
			if (task_deadline.adc>=3){
				log_msg = "adc task missed 3 deadlines\r\n";
				HAL_UART_Transmit_IT(&huart2,(uint8_t *) log_msg, strlen(log_msg));
			}
		}
		else if (ulNotificationValue & I2C_TASK_DEADLINE_MISS_FLAG ){
			task_deadline.i2c++;
			if (task_deadline.i2c>=3){
				log_msg = "i2c task missed 3 deadlines\r\n";
				HAL_UART_Transmit_IT(&huart2,(uint8_t *) log_msg, strlen(log_msg));
			}
		}
		else if (ulNotificationValue & FLOW_CONTROL_TASK_DEADLINE_MISS_FLAG ){
			task_deadline.flow_control++;
			if (task_deadline.flow_control>=3){
				log_msg = "flow control task missed 3 deadlines\r\n";
				HAL_UART_Transmit_IT(&huart2, (uint8_t *)log_msg, strlen(log_msg));
			}
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,50);
	}
	vTaskDelete(NULL);
}

void WatchDogTaskHandler(void* pvParameters){
	EventBits_t uxBits;
	char* log_msg;
	char* deadline_msg;
	for(;;){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		uxBits = xEventGroupWaitBits(wdog_event_group, BIT_0 | BIT_1 | BIT_2 ,
				pdTRUE,pdFALSE,portMAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_IWDG_Refresh(&hiwdg);
		if ( (uxBits & BIT_0) !=0 ){
			log_msg ="bit 0\r\n";
			if (wdog_task[0].lastTick - wdog_task[0].firstTick >wdog_task[0].deadline){
				xTaskNotify( FaultHandlerTask, ADC_TASK_DEADLINE_MISS_FLAG, eSetValueWithOverwrite );
			}
		}
		if ( (uxBits & BIT_1) !=0 ){
			log_msg ="bit 1\r\n";
			if (wdog_task[1].lastTick - wdog_task[1].firstTick >wdog_task[1].deadline){
				xTaskNotify( FaultHandlerTask, I2C_TASK_DEADLINE_MISS_FLAG, eSetValueWithOverwrite );
			}
		}
		if ( (uxBits & BIT_2) !=0 ){
			log_msg ="bit 2\r\n";
			if (wdog_task[2].lastTick - wdog_task[2].firstTick >wdog_task[2].deadline){
				xTaskNotify( FaultHandlerTask, FLOW_CONTROL_TASK_DEADLINE_MISS_FLAG, eSetValueWithOverwrite );
			}
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	vTaskDelete(NULL);
}

void vApplicationIdleHook( void ) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    // Your code goes here.
    // It must NOT call any blocking API functions (e.g., vTaskDelay).
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,char *pcTaskName ){
	taskDISABLE_INTERRUPTS();
	for(;;);
}

