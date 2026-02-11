/*
 * tasks.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */



#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

//#include "event_groups.h"
#include "FreeRTOS.h"

#define FRAME_SIZE	10

#define ADC_QUEUE_SIZE	1000
#define UART_QUEUE_SIZE	100
#define I2C_QUEUE_SIZE	100

#define ADC_MIN_LOTTERY_TICKET 		0
#define ADC_MAX_LOTTERY_TICKET 		30
#define UART_MIN_LOTTERY_TICKET 	31
#define UART_MAX_LOTTERY_TICKET 	34
#define I2C_MIN_LOTTERY_TICKET 		35
#define I2C_MAX_LOTTERY_TICKET 		63

#define ADC_MAX_CONSUMPTION_RATIO 	0.85
#define UART_MAX_CONSUMPTION_RATIO	0.6
#define I2C_MAX_CONSUMPTION_RATIO	0.65

#define ADC_SENSOR_TASK_STACK_SIZE		1024
#define I2C_SENSOR_TASK_STACK_SIZE		1024
#define UART_RECEIVER_TASK_STACK_SIZE	1024
#define FLOW_CONTROL_TASK_STACK_SIZE	1024
#define CONSUMER_TASK_STACK_SIZE		1024
#define FAULT_HANDLER_TASK_STACK_SIZE	1024
#define WATCHDOG_TASK_STACK_SIZE		1024

void I2CSensorTaskHandler(void *pvParameters );
void UARTReceiverTaskHandler(void *pvParameters );
void ADCSensorTaskHandler(void *pvParameters );
void ConsumerTaskHandler(void *pvParameters);
void FlowControlTaskHandler(void *pvParameters );
void FaultTaskHandler(void *pvParameters );
void WatchDogTaskHandler(void* pvParameters);

typedef struct {
    uint8_t length;
    uint8_t data[64];
} frame_t;

typedef enum {
	ADC_OWNERSHIP,
	UART_OWNERSHIP,
	I2C_OWNERSHIP
}production_arbiter_t;

typedef struct {
	uint8_t adc;
	uint8_t i2c;
	uint8_t flow_control;
	uint8_t monitoring;
}task_deadline_counter_t;
#endif /* INC_APP_TASKS_H_ */
