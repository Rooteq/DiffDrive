/*
 * robot.h
 *
 *  Created on: Apr 19, 2024
 *      Author: Rooteq
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "motor.h"

#define SAMPLING_PERIOD 0.01
#define R 40
#define B 85 // TODO: make it more precise

#define KW 5 // TODO: tune
#define IW 0.04

#define KV 1.7
#define IV 0.04

typedef enum
{
	STOPPED = 0,
	MANUAL_MOVE = 1,
	AUTO_MOVE = 2
} InternalState;

typedef struct{
	uint32_t lastTx;
	uint32_t lastPathPlan;
}PollTimers;

typedef uint8_t StateFlag; //

typedef struct{
	int16_t xd; // perhaps float?
	int16_t yd;
	float totalDistanceError;
	float totalAngleError;
}Destination;

typedef struct{
	  float x;
	  float y;
	  float ang;
} Position;

typedef struct
{
	float w;
	float v;

	MotorInstance motorRight;
	MotorInstance motorLeft;

	StateFlag flag;
	Position position;

	bool goToPoint;
	Destination destination;
}Robot;

void initRobot(Robot* robot); // TODO add motor init inside?
void calculatePosition(Robot* robot);

void beginPositionControl(Robot* robot, int16_t x, int16_t y);
void pathPlanner(Robot* robot, PollTimers *timer);

void setStateFlag(Robot* robot, InternalState state);

#endif /* INC_ROBOT_H_ */
