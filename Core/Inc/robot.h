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
#define B 85 // TODO make it more precise

typedef uint8_t ErrorFlag;

typedef struct{ // temporary
	  float x;
	  float y;
	  float ang;
} Position;

typedef struct // equations and everything?
{
	MotorInstance motorRight;
	MotorInstance motorLeft;

	ErrorFlag flag;
	Position position;
}Robot;

void initRobot(Robot* robot); // TODO add motor init inside?
void calculatePosition(Robot* robot);

#endif /* INC_ROBOT_H_ */
