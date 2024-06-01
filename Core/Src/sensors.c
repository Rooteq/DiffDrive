#include "sensors.h"
#include "math.h"

#define I2C_TIMEOUT 50

//float wPrevOutput = 0;
//float wPrevReading = 0;
float alpha = 0.9;

float prevAXOutput = 0;
float prevAYOutput = 0;

float prevAXReading = 0;
float prevAYReading = 0;

I2C_HandleTypeDef *i2c;
float Acc_Scale;
float Gyr_Scale;

void readRadar(Robot* robot, PollTimers* timers, TIM_HandleTypeDef *sensorTimer)
{
	if(HAL_GetTick() - timers->lastRadarPoll > 1500)
	{
		uint32_t start = HAL_TIM_ReadCapturedValue(sensorTimer, TIM_CHANNEL_1);
		uint32_t stop = HAL_TIM_ReadCapturedValue(sensorTimer, TIM_CHANNEL_2);

		float value = (float)(stop - start) / 58.0f;

		if(robot->stopForObstacle == true && value > 20)
		{
			robot->stopForObstacle = false;
			setStateFlag(robot, RESET_OBSTACLE);
		}
		else if(value <= 20)
		{
			robot->stopForObstacle = true;
			robot->obstacleProximity = (uint8_t)value;
			setStateFlag(robot, SET_OBSTACLE);
		}
		timers->lastRadarPoll = HAL_GetTick();
	}
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	i2c = hi2c;
	MPU6050_DeviceReset(1);
    MPU6050_SetSleepEnabled(0);
    MPU6050_SetClockSource(MPU6050_CLOCK_INTERNAL);
    MPU6050_SetDlpf(MPU6050_DLPF_BW_20);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
}

void MPU6050_SetDlpf(uint8_t Value)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Value & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetSleepEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetClockSource(uint8_t Source)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Source & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetFullScaleGyroRange(uint8_t Range)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch(Range)
	{
		case MPU6050_GYRO_FS_250:
			Gyr_Scale = 0.00013322;
//			Gyr_Scale = 0.007633;
			break;
//		case MPU6050_GYRO_FS_500:
//			Gyr_Scale = 0.015267;
//			break;
//		case MPU6050_GYRO_FS_1000:
//			Gyr_Scale = 0.030487;
//			break;
//		case MPU6050_GYRO_FS_2000:
//			Gyr_Scale = 0.060975;
//			break;
		default:
			break;
	}
}

void MPU6050_SetFullScaleAccelRange(uint8_t Range)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch(Range)
	{
		case MPU6050_ACCEL_FS_2:
			//Acc_Scale = 0.000061; // [g]
//			Acc_Scale = 0.0005982; //[m/s^2]
			Acc_Scale = 0.5982; //[mm/s^2]
			break;
//		case MPU6050_ACCEL_FS_4:
//			Acc_Scale = 0.000122;
//			break;
//		case MPU6050_ACCEL_FS_8:
//			Acc_Scale = 0.000244;
//			break;
//		case MPU6050_ACCEL_FS_16:
//			Acc_Scale = 0.0004882;
//			break;
		default:
			break;
	}
}
void MPU6050_GetRotationScaled(float* w)
{
	uint8_t tmp[2];
	//HAL_I2C_Mem_Read_IT(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, tmp, 2);
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);

	int16_t val = ((((int16_t)tmp[0]) << 8) | tmp[1]);
	*w = (float)val*0.00013322-0.021; // offset?
}

float MPU6050_GetAccelerationZ(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	int16_t val = (((int16_t)tmp[0]) << 8) | tmp[1];
	return (float)val * 0.0005982;
}

void MPU6050_GetAccelerometerRAW(int16_t *x, int16_t *y)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
}

void MPU6050_GetAccelerometerScaled(float* x, float* y)
{
	int16_t tmp_x, tmp_y;
	MPU6050_GetAccelerometerRAW(&tmp_x, &tmp_y);

	*x = (float)tmp_x * Acc_Scale  - 190;
	*y = (float)tmp_y * Acc_Scale + 330;
}

void readImu(Robot* robot, PollTimers *timers)
{
	if(HAL_GetTick() - timers->lastImuPoll > 50)
	{
		float w;
		float ax, ay;
		MPU6050_GetRotationScaled(&w);
		MPU6050_GetAccelerometerScaled(&(ax), &(ay));

		float axOutput = alpha * (prevAXOutput + ax - prevAXReading);
		float ayOutput = alpha * (prevAYOutput + ay - prevAYReading);

		robot->imuReadings.w = w;
		robot->imuReadings.ax = axOutput;
		robot->imuReadings.ay = ayOutput;

		prevAXOutput = axOutput;
		prevAYOutput = ayOutput;

		prevAXReading = ax;
		prevAYReading = ay;

		timers->lastImuPoll = HAL_GetTick();
	}
}

