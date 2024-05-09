/*
 * robot.c
 *
 *  Created on: Apr 19, 2024
 *      Author: Rooteq
 */
#include "robot.h"
#include "math.h"

void initRobot(Robot* robot)
{
	robot->flag = 1;
	robot->position.x = 0;
	robot->position.y = 0;
	robot->position.ang = 0; // make it start as magnetometer direction? hybrid?

	robot->goToPoint = false;
	robot->destination.xd = 0;
	robot->destination.yd = 0;
	robot->destination.totalDistanceError = 0;
	robot->destination.totalAngleError = 0;
}

void calculatePosition(Robot* robot) // perhaps store last motor position
{
	// RHS is t-delta(t)
	float leftSpeed = RPM_TO_RAD * robot->motorLeft.rpm;
	float rightSpeed = RPM_TO_RAD * robot->motorRight.rpm;

	robot->position.x = robot->position.x + (SAMPLING_PERIOD * ((R/2.0) * cos(robot->position.ang) * (rightSpeed + leftSpeed)));
	robot->position.y = robot->position.y + (SAMPLING_PERIOD * ((R/2.0) * sin(robot->position.ang) * (rightSpeed + leftSpeed)));
	robot->position.ang = robot->position.ang + (SAMPLING_PERIOD * ((R/(2.0*B)) * (rightSpeed - leftSpeed)));
}

void beginPositionControl(Robot* robot, int16_t x, int16_t y)
{
	robot->destination.xd = x;
	robot->destination.yd = y;
	robot->destination.totalDistanceError = 0;
	robot->destination.totalAngleError = 0;

	robot->goToPoint = true;
}

void pathPlanner(Robot* robot, PollTimers *timer)
{
	if(robot->goToPoint == false) // do not proceed when not in goToPoint mode
		return;

	if(HAL_GetTick() - timer->lastPathPlan > 50) // TODO: look into time
	{
		if((fabsf((float)robot->destination.yd - robot->position.y) < 1) && (fabsf((float)robot->destination.xd - robot->position.x) < 1))
		{
			motorSetSpeed(&(robot->motorLeft), 0);
			motorSetSpeed(&(robot->motorRight), 0);
			robot->goToPoint = false;
			return;
		}

		float dAng = atan2((float)robot->destination.yd - robot->position.y, (float)robot->destination.xd - robot->position.x);
		float ang = (dAng - robot->position.ang);

		robot->destination.totalAngleError += ang;

		float pATerm = (float)KW * ang; // proportional angle term
		float iATerm = (float)IW * robot->destination.totalAngleError * 0.05;
		float w = pATerm + iATerm; // + iATerm

		float distance = sqrt(pow(((float)robot->destination.xd - robot->position.x),2)+pow(((float)robot->destination.yd - robot->position.y),2));

		robot->destination.totalDistanceError += distance;

		float pVTerm = (float)KV * distance;
		float iVTerm = (float)IV * robot->destination.totalDistanceError * 0.05;

		float v = pVTerm + iVTerm;

		robot->w = w;
		robot->v = v;

		float lWheelSpeed = (1/(float)R)*v - ((float)B*2.0)/(2.0*(float)R)*w;
		float rWheelSpeed = (1/(float)R)*v + ((float)B*2.0)/(2.0*(float)R)*w;

		if(lWheelSpeed > 10) // clamping
			lWheelSpeed = 10;
		if(lWheelSpeed < -10)
			lWheelSpeed = -10;

		if(rWheelSpeed > 10)
			rWheelSpeed = 10;
		if(rWheelSpeed < -10)
			rWheelSpeed = -10;

		motorContinousSetSpeed(&(robot->motorLeft), lWheelSpeed);
		motorContinousSetSpeed(&(robot->motorRight), rWheelSpeed);

		timer->lastPathPlan = HAL_GetTick();
	}


}
