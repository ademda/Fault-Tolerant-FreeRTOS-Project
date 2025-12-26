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

void SensorTaskHandler(void *pvParameters ){
    for (;;)
    {

    }
    vTaskDelete(NULL);
}
void ControlTaskHandler(void *pvParameters ){
	for (;;)
	{

	}
	vTaskDelete(NULL);

}
void MonitorTaskHandler(void *pvParameters ){
	for (;;)
	{

	}
	vTaskDelete(NULL);
}
void ComTaskHandler(void *pvParameters ){
	for (;;)
	{

	}
	vTaskDelete(NULL);
}
void FaultTaskHandler(void *pvParameters ){
	for (;;)
	{

	}
	vTaskDelete(NULL);
}
