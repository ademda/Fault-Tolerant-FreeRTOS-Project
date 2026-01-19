/*
 * sensors.h
 *
 *  Created on: Dec 28, 2025
 *      Author: dalya
 */

#ifndef INC_ADC_SENSOR_H_
#define INC_ADC_SENSOR_H_

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

typedef struct {
	uint8_t accRxBuf[8];
	uint8_t gyroRxBuf[8];
} imu_data_t;


#endif /* INC_ADC_SENSOR_H_ */
