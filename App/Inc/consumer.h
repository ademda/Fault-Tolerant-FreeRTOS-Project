/*
 * consumer.h
 *
 *  Created on: Dec 28, 2025
 *      Author: dalya
 */

#ifndef INC_CONSUMER_H_
#define INC_CONSUMER_H_

typedef struct {
	float *pdata;
	double timestamp;
	uint16_t priority;
	uint8_t lenght;
} sensor_data_t;

#endif /* INC_CONSUMER_H_ */
