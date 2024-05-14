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

void initMotor(MotorInstance* motor, TIM_HandleTypeDef *motorHtim, uint8_t motorChannel, TIM_HandleTypeDef *encoderHtim, MotorSide side)
{
	motor->stop = true;
	motor->side = side;
	motor->setRpm = 0;
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

void motorRegulateVelocity(MotorInstance* motor)
{
	if(motor->stop == true)
	{
		motorSetDirection(motor, STOP);
		return;
	}

	motor->currentPWM += pid_calculate(&(motor->pid_controller), motor->setRpm, motor->rpm);

	if(motor->currentPWM > 600)
		motor->currentPWM = 600;
	if(motor->currentPWM < -600)
		motor->currentPWM = -600;

	if(motor->currentPWM >= 0)
	{
		motorSetDirection(motor, FORWARDS);
		if(motor->side == RIGHT)
			__HAL_TIM_SET_COMPARE(motor->motorHtim, TIM_CHANNEL_1, motor->currentPWM);
		else
			__HAL_TIM_SET_COMPARE(motor->motorHtim, TIM_CHANNEL_2, motor->currentPWM);

	}
	else
	{
		motorSetDirection(motor, BACKWARDS);
		if(motor->side == RIGHT)
			__HAL_TIM_SET_COMPARE(motor->motorHtim, TIM_CHANNEL_1, -motor->currentPWM);
		else
			__HAL_TIM_SET_COMPARE(motor->motorHtim, TIM_CHANNEL_2, -motor->currentPWM);

	}
}

void motorSetConstSpeed(MotorInstance* motor, float setRpm)
{
	if(setRpm == 0)
	{
		motor->stop = true;
		motor->setRpm = setRpm;
		motor->currentPWM = 0; // very important, without it not always pulls to 0
		return;
	}

	if(motor->setRpm != setRpm)
	{
		motor->stop = false;
		motor->setRpm = setRpm;
		pid_reset(&(motor->pid_controller));
	}
}

void motorContinousSetSpeed(MotorInstance* motor, float setRpm)
{
	if(setRpm == 0)
	{
		motor->stop = true;
		motor->setRpm = setRpm;
		motor->currentPWM = 0; // very important - without it, it sometimes isnt 0
		return;
	}

	if(motor->setRpm != setRpm)
	{
		motor->stop = false;
		motor->setRpm = setRpm;
	}
}


void motorSetDirection(MotorInstance* motor, MotorDirection direction)
{
	if(direction == STOP)
	{
		HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
		return;
	}

	if(direction == FORWARDS)
	{
		if(motor->side == RIGHT)
		{
			HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
		}
		else if(motor->side == LEFT)
		{
			HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
		}
	}
	else if(direction == BACKWARDS)
	{
		if(motor->side == RIGHT)
		{
			HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
		}
		else if(motor->side == LEFT)
		{
			HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
		}
	}
}


