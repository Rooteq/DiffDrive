/*
 * sensors.h
 *
 *  Created on: May 14, 2024
 *      Author: Rooteq
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <main.h>
#include "robot.h"

#define MPU6050_ADDRESS 			0xD0
#define MPU6050_CLOCK_INTERNAL      0x00
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_ZOUT_H      0x47

#define MPU6050_PWR1_DEVICE_RESET_BIT   7

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_DeviceReset(uint8_t Reset);
void MPU6050_SetSleepEnabled(uint8_t Enable);
void MPU6050_SetClockSource(uint8_t Source);
void MPU6050_SetDlpf(uint8_t Value);

void readRadar(Robot* robot, PollTimers* timers, TIM_HandleTypeDef *sensorTimer);
void MPU6050_SetIntEnableRegister(uint8_t Value);

void MPU6050_SetFullScaleGyroRange(uint8_t Range);
void MPU6050_SetFullScaleAccelRange(uint8_t Range);

float MPU6050_GetRotationScaled(void);

int16_t MPU6050_GetAccelerationXRAW(void);
int16_t MPU6050_GetAccelerationYRAW(void);
int16_t MPU6050_GetAccelerationZRAW(void);
void MPU6050_GetAccelerometerRAW(int16_t* x, int16_t* y, int16_t* z);
void MPU6050_GetAccelerometerScaled(float* x, float* y, float* z);

#endif /* INC_SENSORS_H_ */
