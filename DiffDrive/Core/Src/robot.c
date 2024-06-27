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
	setStateFlag(robot, STOPPED);
	robot->position.x = 0;
	robot->position.y = 0;
	robot->position.ang = 0;

	robot->encoderCalculatedPosition.x = 0;
	robot->encoderCalculatedPosition.y = 0;
	robot->encoderCalculatedPosition.ang = 0;

	robot->goToPoint = false;
	robot->stopForObstacle = false;

	robot->destination.xd = 0;
	robot->destination.yd = 0;
	robot->destination.totalDistanceError = 0;
	robot->destination.totalAngleError = 0;

	robot->imuReadings.w = 0;
	robot->imuReadings.ax = 0;
	robot->imuReadings.ay = 0;
	robot->imuReadings.vxGlobal = 0;
	robot->imuReadings.vyGlobal = 0;
}

void calculatePosition(Robot* robot)
{
	float leftSpeed = RPM_TO_RAD * robot->motorLeft.rpm;
	float rightSpeed = RPM_TO_RAD * robot->motorRight.rpm;

	robot->encoderCalculatedPosition.x = robot->position.x + (SAMPLING_PERIOD * ((R/2.0) * cos(robot->position.ang) * (rightSpeed + leftSpeed)));
	robot->encoderCalculatedPosition.y = robot->position.y + (SAMPLING_PERIOD * ((R/2.0) * sin(robot->position.ang) * (rightSpeed + leftSpeed)));
	robot->encoderCalculatedPosition.ang = robot->position.ang + (SAMPLING_PERIOD * ((R/(2.0*B)) * (rightSpeed - leftSpeed)));

	// x and y are reversed, because of chassis mounting
	float axGlobal = robot->imuReadings.ay*cos(robot->position.ang) - robot->imuReadings.ax*sin(robot->position.ang);
	float ayGlobal = robot->imuReadings.ay*sin(robot->position.ang) + robot->imuReadings.ax*cos(robot->position.ang);
//
	robot->imuReadings.vxGlobal += axGlobal * SAMPLING_PERIOD;
	robot->imuReadings.vyGlobal += ayGlobal * SAMPLING_PERIOD;

	float accelX = robot->imuReadings.vxGlobal * SAMPLING_PERIOD;
	float accelY = robot->imuReadings.vyGlobal * SAMPLING_PERIOD;
//
//    float deltaX_odo = SAMPLING_PERIOD * ((R / 2.0) * cos(robot->position.ang) * (rightSpeed + leftSpeed));
//    float deltaY_odo = SAMPLING_PERIOD * ((R / 2.0) * sin(robot->position.ang) * (rightSpeed + leftSpeed));

	// position calculated by encoder is treated as absolute
//	robot->position.x = 0.98 * robot->encoderCalculatedPosition.x + 0.02 * (accelX + robot->position.x);
//	robot->position.y = 0.98 * robot->encoderCalculatedPosition.y + 0.02 * (accelY + robot->position.y);
//	robot->position.x += accelX;
//	robot->position.y += accelY;


	robot->position.x = robot->encoderCalculatedPosition.x;
	robot->position.y = robot->encoderCalculatedPosition.y;
//	robot->position.ang = robot->encoderCalculatedPosition.ang;
	robot->position.ang = 0.98 * robot->encoderCalculatedPosition.ang + 0.02 * (robot->imuReadings.w * SAMPLING_PERIOD + robot->position.ang);
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

	setStateFlag(robot, AUTO_MOVE);
	if(HAL_GetTick() - timer->lastPathPlan > 200)
	{
		if((fabsf((float)robot->destination.yd - robot->position.y) < 3) && (fabsf((float)robot->destination.xd - robot->position.x) < 3))
		{
			motorSetConstSpeed(&(robot->motorLeft), 0);
			motorSetConstSpeed(&(robot->motorRight), 0);
			robot->goToPoint = false;
			setStateFlag(robot, STOPPED);
			return;
		}

		float dAng = atan2((float)robot->destination.yd - robot->position.y, (float)robot->destination.xd - robot->position.x);
		float ang = (dAng - robot->position.ang);

		robot->destination.totalAngleError += ang;

		float pATerm = (float)KW * ang; // proportional angle term
		float iATerm = (float)IW * robot->destination.totalAngleError * 0.2;
		float w = pATerm + iATerm; // + iATerm

		float distance = sqrt(pow(((float)robot->destination.xd - robot->position.x),2)+pow(((float)robot->destination.yd - robot->position.y),2));

		robot->destination.totalDistanceError += distance;

		float pVTerm = (float)KV * distance;

		if(pVTerm > 100)
			pVTerm = 100;

		float iVTerm = (float)IV * robot->destination.totalDistanceError * 0.2;

		if(iVTerm > 100)
			iVTerm = 100;

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

void setStateFlag(Robot* robot, InternalState state) // TODO change logic
{
	switch (state) {
		case STOPPED:
			robot->flag = 0x03;
			break;
		case MANUAL_MOVE:
			robot->flag = 0x05;
			break;
		case AUTO_MOVE:
			robot->flag = 0x09;
			break;
		case SET_OBSTACLE:
			robot->flag |= 0x10;
			break;
		case RESET_OBSTACLE:
			robot->flag ^= 0x10;
			break;
		default:
			break;
	}

	if(robot->stopForObstacle == true)
	{
		robot->flag |= 0x10;
	}
}


void initPollTimers(PollTimers* timers)
{
	timers->lastTx = HAL_GetTick();
	timers->lastPathPlan = HAL_GetTick();
	timers->lastRadarPoll = HAL_GetTick();
	timers->lastImuPoll = HAL_GetTick();
}
