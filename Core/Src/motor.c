#include "motor.h"

void updateEncoder(encoderInstance* encoder, TIM_HandleTypeDef *htim)
{
	uint32_t tmpCounter = __HAL_TIM_GET_COUNTER(htim);
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
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder->velocity = -encoder->lastCounterValue - (__HAL_TIM_GET_AUTORELOAD(htim) - tmpCounter);
			}
			else
			{
				encoder->velocity = tmpCounter - encoder->lastCounterValue;
			}
		}
		else
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder->velocity = tmpCounter - encoder->lastCounterValue;
			}
			else
			{
				encoder->velocity = tmpCounter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder->lastCounterValue);
			}
		}
	}
	encoder->position += encoder->velocity; // TODO: convert to w/s? and radians
	encoder->lastCounterValue = tmpCounter;

	encoder->w = 0.17454*encoder->velocity;
	encoder->alfa += encoder->w*0.01;
}

void resetEncoder(encoderInstance* encoder)
{
	encoder->velocity = 0;
	encoder->position= 0;
	encoder->lastCounterValue = 0;

	encoder->w = 0;
	encoder->alfa = 0;
}
