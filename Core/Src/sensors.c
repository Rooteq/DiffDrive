#include "sensors.h"

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
