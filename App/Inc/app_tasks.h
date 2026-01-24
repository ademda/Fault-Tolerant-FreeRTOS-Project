/*
 * tasks.h
 *
 *  Created on: Dec 26, 2025
 *      Author: dalya
 */

#ifndef INC_APP_TASKS_H_
#define INC_APP_TASKS_H_

#define FRAME_SIZE	64

void I2CSensorTaskHandler(void *pvParameters );
void UARTReceiverTaskHandler(void *pvParameters );
void ADCSensorTaskHandler(void *pvParameters );
void ConsumerTaskHandler(void *pvParameters);
void InterfaceTaskHandler(void *pvParameters);
void MonitorTaskHandler(void *pvParameters );
void FlowControlTaskHandler(void *pvParameters );
void FaultTaskHandler(void *pvParameters );

typedef struct{
	uint8_t length;
	uint8_t data[FRAME_SIZE];
}frame_t;

#endif /* INC_APP_TASKS_H_ */
