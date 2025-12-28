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
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;


void SPISensorTaskHandler(void *pvParameters ){
	for (;;){

	}
	vTaskDelete(NULL);
}

void UARTReceiverTaskHandler(void *pvParameters ){
	for (;;){

	}
	vTaskDelete(NULL);
}

void ADCSensorTaskHandler(void *pvParameters ){
	for (;;){

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
