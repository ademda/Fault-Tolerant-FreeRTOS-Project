/*
 * tasks.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#define FRAME_SIZE	10

#define ADC_QUEUE_SIZE	1000
#define UART_QUEUE_SIZE	100
#define I2C_QUEUE_SIZE	100

#define ADC_MIN_LOTTERY_TICKET 		0
#define ADC_MAX_LOTTERY_TICKET 		25
#define UART_MIN_LOTTERY_TICKET 	26
#define UART_MAX_LOTTERY_TICKET 	35
#define I2C_MIN_LOTTERY_TICKET 		36
#define I2C_MAX_LOTTERY_TICKET 		63

#define ADC_MAX_CONSUMPTION_RATIO 	0.85
#define UART_MAX_CONSUMPTION_RATIO	0.6
#define I2C_MAX_CONSUMPTION_RATIO	0.65

void I2CSensorTaskHandler(void *pvParameters );
void UARTReceiverTaskHandler(void *pvParameters );
void ADCSensorTaskHandler(void *pvParameters );
void ConsumerTaskHandler(void *pvParameters);
void MonitorTaskHandler(void *pvParameters );
void FlowControlTaskHandler(void *pvParameters );
void FaultTaskHandler(void *pvParameters );

typedef struct{
	uint8_t length;
	float data[FRAME_SIZE];
}frame_t;

typedef enum {
	ADC_OWNERSHIP,
	UART_OWNERSHIP,
	I2C_OWNERSHIP
}production_arbiter_t;

#endif /* INC_APP_TASKS_H_ */
