/*
 * mpu6050.c
 *
 *  Created on: Oct 16, 2025
 *      Author: dalya
 */

#include <stdlib.h>
#include "main.h"
#include "mpu6050.h"
#include <string.h>
#include "math.h"

#define G 9.81

extern I2C_HandleTypeDef hi2c1;
float Gx_sum = 0, Gy_sum = 0, Gz_sum = 0;
const int NUM_SAMPLES = 500;
uint16_t calib_iter = 0;
MPU6050_t imu;
MPU6050_queue_item_t imu_queue_item;


HAL_StatusTypeDef MPU6050_Init(MPU6050_t *imu){

	imu->Ax = 0;
	imu->Ay = 0;
	imu->Az = 0;
	imu->Gx = 0;
	imu->Gy = 0;
	imu->Gz = 0;
	imu->Gxang = 0;
	imu->Gyang = 0;
	imu->Gzang = 0;
	imu->Gxvelbias = 0;
	imu->Gyvelbias = 0;
	imu->Gzvelbias = 0;
	memset(imu->accRxBuf, 0, sizeof(imu->accRxBuf));
	memset(imu->gyroRxBuf, 0, sizeof(imu->gyroRxBuf));
	memset(imu->imuRxBuf, 0, sizeof(imu->imuRxBuf));
	imu->i2cHandle = &hi2c1;
	imu->pitch = 0;
	imu->roll = 0;
	imu->state = MPU6050_IDLE;
	imu->calib_status = MPU6050_CALIBRATED;
	
	uint8_t check = 0;
	uint8_t Data = 0;
	if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000)){
		Error_Handler();
	}


	//select internal clock and wake up the sensor
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000)){
		Error_Handler();
	}
	// Set accelerometer configuration in ACCEL_CONFIG Register
	Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> <strong>±</strong> 2g
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000)){
		Error_Handler();
	}

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> <strong>±</strong> 250 ̐/s
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000)){
		Error_Handler();
	}

    Data = 0x01;  // DLPF_CFG = 3 (44Hz bandwidth, 1kHz output rate)
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000)){
        Error_Handler();
    }
    
    Data = 0x09;  // SMPLRT_DIV = 9 for 100Hz sampling //13 for 50Hz
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000)){
        Error_Handler();
    }

	/*Data = 0x01;
	//Enable MPU6050 INT interrupt
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, IT_ENABLE_REG, 1, &Data, 1, 1000)){
		Error_Handler();
	}*/
	return HAL_OK;
}

void MPU6050_Read_Accel(MPU6050_t *imu){
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);

	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	float Ax = (float)Accel_X_RAW / 16384.0 * G;
	float Ay = (float)Accel_Y_RAW / 16384.0 * G;
	float Az = (float)Accel_Z_RAW / 16384.0 * G;

	// Compute tilt angles (in degrees)
	imu->roll  = atan2f(Ay, sqrtf(Ax * Ax + Az * Az));  
	imu->pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)); 
	imu->Ax = Ax;
	imu->Ay = Ay;
	imu->Az = Az;
}

void MPU6050_Read_Gyro(MPU6050_t *imu){

	uint8_t Rec_Data[6] ;
	memset(Rec_Data, 0,sizeof(Rec_Data));
	if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000)){
		Error_Handler();
	}

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	float Gx = (float)Gyro_X_RAW / 131.0;
	float Gy = (float)Gyro_Y_RAW / 131.0;
	float Gz = (float)Gyro_Z_RAW / 131.0;
	
	if (imu->calib_status == MPU6050_CALIBRATING){
		calib_iter++;
		Gx_sum += Gx;
        Gy_sum += Gy;
        Gz_sum += Gz;
		if (calib_iter >= NUM_SAMPLES){
			imu->Gxvelbias = Gx_sum / NUM_SAMPLES;
    		imu->Gyvelbias = Gy_sum / NUM_SAMPLES;
    		imu->Gzvelbias = Gz_sum / NUM_SAMPLES;
			imu->calib_status = MPU6050_CALIBRATED;
		}
	}
	else if (imu->calib_status == MPU6050_CALIBRATED){
		imu->Gx = (Gx - imu->Gxvelbias)*(M_PI / 180.0f);
		imu->Gy = (Gy - imu->Gyvelbias)*(M_PI / 180.0f);
		imu->Gz = (Gz - imu->Gzvelbias)*(M_PI / 180.0f);
		float dt = 0.01; // example: if sampling at 100 Hz

		imu->Gxang += imu->Gx * dt;
		imu->Gyang += imu->Gy * dt;
		imu->Gzang += imu->Gz * dt;

	}
	
}


HAL_StatusTypeDef MPU6050_Read_Accel_DMA(MPU6050_t *imu){
	if (imu->i2cHandle->State != HAL_I2C_STATE_READY) {
	    return HAL_BUSY;
	}
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, MPU6050_ADDR, 0x3B, 1, imu->accRxBuf, 6)){
		return HAL_ERROR;
	}
	return HAL_OK;
}



void MPU6050_Read_Accel_DMA_Complete(MPU6050_t *imu){
	int16_t Accel_X_RAW = (int16_t)(imu->accRxBuf[0] << 8 | imu->accRxBuf [1]);
	int16_t Accel_Y_RAW = (int16_t)(imu->accRxBuf[2] << 8 | imu->accRxBuf [3]);
	int16_t Accel_Z_RAW = (int16_t)(imu->accRxBuf[4] << 8 | imu->accRxBuf [5]);

	float Ax = (float)Accel_X_RAW / 16384.0 * G;
	float Ay = (float)Accel_Y_RAW / 16384.0 * G;
	float Az = (float)Accel_Z_RAW / 16384.0 * G;

	// Compute tilt angles (in degrees)
	imu->roll = atan2f(Ay, sqrtf(Ax * Ax + Az * Az));
	imu->pitch  = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
	imu->Ax = Ax;
	imu->Ay = Ay;
	imu->Az = Az;
}

HAL_StatusTypeDef MPU6050_Read_Gyro_DMA(MPU6050_t *imu){
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, MPU6050_ADDR, 0x43, 1, imu->gyroRxBuf, 6)){
		return HAL_ERROR;
	}
	return HAL_OK;
}

void MPU6050_Read_Gyro_DMA_Complete(MPU6050_t *imu){
	uint8_t Rec_Data[6] ;
	memset(Rec_Data, 0,sizeof(Rec_Data));

	int16_t Gyro_X_RAW = (int16_t)(imu->gyroRxBuf[0] << 8 | imu->gyroRxBuf [1]);
	int16_t Gyro_Y_RAW = (int16_t)(imu->gyroRxBuf[2] << 8 | imu->gyroRxBuf [3]);
	int16_t Gyro_Z_RAW = (int16_t)(imu->gyroRxBuf[4] << 8 | imu->gyroRxBuf [5]);

	float Gx = (float)Gyro_X_RAW / 131.0;
	float Gy = (float)Gyro_Y_RAW / 131.0;
	float Gz = (float)Gyro_Z_RAW / 131.0;

	if (imu->calib_status == MPU6050_CALIBRATING){
		calib_iter++;
		Gx_sum += Gx;
        Gy_sum += Gy;
        Gz_sum += Gz;
		if (calib_iter >= NUM_SAMPLES){
			imu->Gxvelbias = Gx_sum / NUM_SAMPLES;
    		imu->Gyvelbias = Gy_sum / NUM_SAMPLES;
    		imu->Gzvelbias = Gz_sum / NUM_SAMPLES;
			imu->calib_status = MPU6050_CALIBRATED;
		}
	}
	else if (imu->calib_status == MPU6050_CALIBRATED){
		imu->Gx = (Gx - imu->Gxvelbias)*(M_PI / 180.0f);
		imu->Gy = (Gy - imu->Gyvelbias)*(M_PI / 180.0f);
		imu->Gz = (Gz - imu->Gzvelbias)*(M_PI / 180.0f);
		float dt = 0.01; // example: if sampling at 100 Hz

		imu->Gxang += imu->Gx * dt;
		imu->Gyang += imu->Gy * dt;
		imu->Gzang += imu->Gz * dt;
	}
}



void MPU6050_Read_IMU(MPU6050_t *imu){
	uint8_t Rec_Data[14];
	if (HAL_I2C_Mem_Read(imu->i2cHandle, MPU6050_ADDR, 0x3B, 1, Rec_Data, 16, 1000) != HAL_OK){
		Error_Handler();
	}
	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	float Ax = (float)Accel_X_RAW / 16384.0 * G;
	float Ay = (float)Accel_Y_RAW / 16384.0 * G;
	float Az = (float)Accel_Z_RAW / 16384.0 * G;

	// Compute tilt angles (in degrees)
	imu->roll  = atan2f(Ay, sqrtf(Ax * Ax + Az * Az));
	imu->pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
	imu->Ax = Ax;
	imu->Ay = Ay;
	imu->Az = Az;

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data [9]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data [11]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data [13]);

	float Gx = (float)Gyro_X_RAW / 131.0;
	float Gy = (float)Gyro_Y_RAW / 131.0;
	float Gz = (float)Gyro_Z_RAW / 131.0;

	imu->Gx = (Gx - imu->Gxvelbias)*(M_PI / 180.0f);
	imu->Gy = (Gy - imu->Gyvelbias)*(M_PI / 180.0f);
	imu->Gz = (Gz - imu->Gzvelbias)*(M_PI / 180.0f);
	float dt = 0.01; // example: if sampling at 100 Hz

	imu->Gxang += imu->Gx * dt;
	imu->Gyang += imu->Gy * dt;
	imu->Gzang += imu->Gz * dt;

}

HAL_StatusTypeDef MPU6050_Read_IMU_DMA(MPU6050_t *imu){
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, MPU6050_ADDR, 0x3B, 1, imu->imuRxBuf, 16)){
		return HAL_ERROR;
	}
	return HAL_OK;
}

void MPU6050_Read_IMU_DMA_Complete(MPU6050_t *imu){
	int16_t Accel_X_RAW = (int16_t)(imu->imuRxBuf[0] << 8 | imu->imuRxBuf[1]);
	int16_t Accel_Y_RAW = (int16_t)(imu->imuRxBuf[2] << 8 | imu->imuRxBuf[3]);
	int16_t Accel_Z_RAW = (int16_t)(imu->imuRxBuf[4] << 8 | imu->imuRxBuf[5]);

	float Ax = (float)Accel_X_RAW / 16384.0 * G;
	float Ay = (float)Accel_Y_RAW / 16384.0 * G;
	float Az = (float)Accel_Z_RAW / 16384.0 * G;

	// Compute tilt angles (in degrees)
	imu->roll  = atan2f(Ay, sqrtf(Ax * Ax + Az * Az));
	imu->pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
	imu->Ax = Ax;
	imu->Ay = Ay;
	imu->Az = Az;

	int16_t Gyro_X_RAW = (int16_t)(imu->imuRxBuf[8] << 8 | imu->imuRxBuf [9]);
	int16_t Gyro_Y_RAW = (int16_t)(imu->imuRxBuf[10] << 8 | imu->imuRxBuf [11]);
	int16_t Gyro_Z_RAW = (int16_t)(imu->imuRxBuf[12] << 8 | imu->imuRxBuf [13]);

	float Gx = (float)Gyro_X_RAW / 131.0;
	float Gy = (float)Gyro_Y_RAW / 131.0;
	float Gz = (float)Gyro_Z_RAW / 131.0;

	imu->Gx = (Gx - imu->Gxvelbias)*(M_PI / 180.0f);
	imu->Gy = (Gy - imu->Gyvelbias)*(M_PI / 180.0f);
	imu->Gz = (Gz - imu->Gzvelbias)*(M_PI / 180.0f);
	float dt = 0.01; // example: if sampling at 100 Hz

	imu->Gxang += imu->Gx * dt;
	imu->Gyang += imu->Gy * dt;
	imu->Gzang += imu->Gz * dt;
}

/*void Calibrate_Gyro(MPU6050_t *imu){
    float Gx_sum = 0, Gy_sum = 0, Gz_sum = 0;
    const int NUM_SAMPLES = 50;  // More samples for better accuracy
    
    // Reset biases to zero during calibration
    imu->Gxvelbias = 0;
    imu->Gyvelbias = 0; 
    imu->Gzvelbias = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++){
        uint8_t Rec_Data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);
        
        // Get RAW angular velocities (no bias correction yet)
        int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        
        // Convert to degrees/sec
        float Gx = (float)Gyro_X_RAW / 131.0;
        float Gy = (float)Gyro_Y_RAW / 131.0;
        float Gz = (float)Gyro_Z_RAW / 131.0;
        
        Gx_sum += Gx;
        Gy_sum += Gy;
        Gz_sum += Gz;
        
        HAL_Delay(5);  // Small delay between samples
    }
    
    // Calculate average bias
    imu->Gxvelbias = Gx_sum / NUM_SAMPLES;
    imu->Gyvelbias = Gy_sum / NUM_SAMPLES;
    imu->Gzvelbias = Gz_sum / NUM_SAMPLES;
}*/

HAL_StatusTypeDef MPU6050_ClearInterrupt(MPU6050_t *imu){
	uint8_t intStatus ;
	// Read INT_STATUS register to clear interrupt
	if (HAL_I2C_Mem_Read(imu->i2cHandle, MPU6050_ADDR, 0x3A, 1, &intStatus, 1, 1000) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_OK;
}
