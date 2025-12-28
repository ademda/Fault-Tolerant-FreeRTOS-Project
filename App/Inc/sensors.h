/*
 * sensors.h
 *
 *  Created on: Dec 28, 2025
 *      Author: dalya
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

typedef struct {
	uint8_t accRxBuf[8];
	uint8_t gyroRxBuf[8];
} imu_data_t;

#endif /* INC_SENSORS_H_ */
