/*
 * mpu6050.h
 *
 *  Created on: Oct 16, 2025
 *      Author: dalya
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 		0xD0
#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_1_REG		0x6B
#define SMPLRT_DIV_REG		0x19
#define CONFIG_REG			0x1A
#define ACCEL_CONFIG_REG	0x1C
#define GYRO_CONFIG_REG		0x1B
#define IT_ENABLE_REG		0x38

#define WHO_AM_I_RESP 0x68

typedef enum {
    MPU6050_IDLE,
	MPU6050_ACCEL_READING,
    MPU6050_ACCEL_READING_CMPLT,
	MPU6050_GYRO_READING,
	MPU6050_GYRO_READING_CMPLT
} MPU6050_State_t;

typedef enum {
    MPU6050_CALIBRATING,
	MPU6050_CALIBRATED,
} MPU6050_CalibState_t;

typedef struct{
	/* I2C Typedef	*/
	I2C_HandleTypeDef *i2cHandle;
	/*	DMA VALS	*/
	uint8_t accRxBuf[8];
	uint8_t gyroRxBuf[8];
	uint8_t imuRxBuf[14];
	/*	POLLING VALS*/
	float pitch;
	float roll;
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	float Gxang;
	float Gyang;
	float Gzang;
	float Gxvelbias;
	float Gyvelbias;
	float Gzvelbias;
	/*Reading status*/
	MPU6050_State_t state;
	/*Calibratint status*/
	MPU6050_CalibState_t calib_status;
} MPU6050_t;

typedef struct {
	float pitch;
	float roll;
} MPU6050_queue_item_t;

typedef struct {
	uint8_t accRxBuf[8];
	uint8_t gyroRxBuf[8];
} imu_data_t;


HAL_StatusTypeDef MPU6050_Init(MPU6050_t *imu);

void MPU6050_Read_Accel(MPU6050_t *imu);

void MPU6050_Read_Gyro(MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_Read_Accel_DMA(MPU6050_t *imu);

void MPU6050_Read_Accel_DMA_Complete(MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_Read_Gyro_DMA(MPU6050_t *imu);

void MPU6050_Read_Gyro_DMA_Complete(MPU6050_t *imu);

void MPU6050_Read_IMU(MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_Read_IMU_DMA(MPU6050_t *imu);

void MPU6050_Read_IMU_DMA_Complete(MPU6050_t *imu);

HAL_StatusTypeDef MPU6050_ClearInterrupt(MPU6050_t *imu);

void Calibrate_Gyro(MPU6050_t *imu);

#endif /* INC_MPU6050_H_ */
