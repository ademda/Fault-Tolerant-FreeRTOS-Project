/*
 * tasks.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#include "main.h"

void I2CSensorTaskHandler(void *pvParameters );
void UARTReceiverTaskHandler(void *pvParameters );
void ADCSensorTaskHandler(void *pvParameters );
void ConsumerTaskHandler(void *pvParameters);
void InterfaceTaskHandler(void *pvParameters);
void MonitorTaskHandler(void *pvParameters );
void FlowControlTaskHandler(void *pvParameters );
void FaultTaskHandler(void *pvParameters );

#endif /* INC_APP_TASKS_H_ */
