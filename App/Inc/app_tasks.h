/*
 * tasks.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#include "main.h"

void SensorTaskHandler(void *pvParameters );
void ControlTaskHandler(void *pvParameters );
void MonitorTaskHandler(void *pvParameters );
void ComTaskHandler(void *pvParameters );
void FaultTaskHandler(void *pvParameters );

#endif /* INC_APP_TASKS_H_ */
