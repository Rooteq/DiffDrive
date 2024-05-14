/*
 * sensors.h
 *
 *  Created on: May 14, 2024
 *      Author: Rooteq
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <main.h>
#include "robot.h"

void readRadar(Robot* robot, PollTimers* timers, TIM_HandleTypeDef *sensorTimer);

#endif /* INC_SENSORS_H_ */
