#include "motor.h"

void motorUpdateVelocity(MotorInstance* motor)
{
	EncoderInstance* encoder = &(motor->encoder);

	uint32_t tmpCounter = __HAL_TIM_GET_COUNTER(encoder->encoderHtim);
	static uint8_t firstTime = 0;

	if(!firstTime)
	{
		encoder->velocity = 0;
		firstTime = 1;
	}
	else
	{
		if(tmpCounter == encoder->lastCounterValue)
		{
			encoder->velocity = 0;
		}
		else if(tmpCounter > encoder->lastCounterValue)
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->encoderHtim))
			{
				encoder->velocity = -encoder->lastCounterValue - (__HAL_TIM_GET_AUTORELOAD(encoder->encoderHtim) - tmpCounter);
			}
			else
			{
				encoder->velocity = tmpCounter - encoder->lastCounterValue;
			}
		}
		else
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->encoderHtim))
			{
				encoder->velocity = tmpCounter - encoder->lastCounterValue;
			}
			else
			{
				encoder->velocity = tmpCounter + (__HAL_TIM_GET_AUTORELOAD(encoder->encoderHtim) - encoder->lastCounterValue);
			}
		}
	}
	encoder->lastCounterValue = tmpCounter;

	motor->position += encoder->velocity;
	motor->rpm = encoder->velocity*1.67;
}

void initMotor(MotorInstance* motor, TIM_HandleTypeDef *motorHtim, uint8_t motorChannel, TIM_HandleTypeDef *encoderHtim)
{
	motor->currentPWM = 0;
	motor->motorHtim = motorHtim;
	motor->motorChannel = motorChannel;
	motor->rpm = 0;
	motor->position=0;

	motor->encoder.lastCounterValue = 0;
	motor->encoder.encoderHtim = encoderHtim;
	motor->encoder.velocity = 0;

	pid_init(&(motor->pid_controller), MOTOR_A_Kp, MOTOR_A_Ki, MOTOR_A_Kd, MOTOR_A_ANTI_WINDUP);

	__HAL_TIM_SET_COMPARE(motor->motorHtim, motor->motorChannel, motor->currentPWM);
}

void motorSetSpeed(MotorInstance* motor, int setSpeed)
{
	int output = pid_calculate(&(motor->pid_controller), setSpeed, motor->rpm);

	motor->currentPWM += output;

	if(motor->currentPWM > 1000)
	{
		return;
	}

	if(motor->currentPWM >= 0)
	{
		 __HAL_TIM_SET_COMPARE(motor->motorHtim, motor->motorChannel, motor->currentPWM);
	}
	else
	{
		 __HAL_TIM_SET_COMPARE(motor->motorHtim, motor->motorChannel, 0);
	}
}

