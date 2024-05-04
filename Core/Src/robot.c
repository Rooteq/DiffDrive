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
