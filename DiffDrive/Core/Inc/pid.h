/*
 * pid.h
 *
 *  Created on: Apr 13, 2024
 *      Author: Rooteq
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define SAMPLING_RATE 100
#define PID_MAX_INTEGRAL 20000

typedef struct
{
	int previous_error;
	int total_error;
	float Kp;
	float Ki;
	float Kd;
	int anti_windup_limit;
}pid_str;

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init);
void pid_reset(pid_str *pid_data);

int pid_calculate(pid_str *pid_data, int setpoint, float process_variable);


#endif /* INC_PID_H_ */
