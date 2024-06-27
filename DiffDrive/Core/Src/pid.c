/*
 * pid.c
 *
 *  Created on: Apr 13, 2024
 *      Author: Rooteq
 */
#include "pid.h"

void pid_init(pid_str *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
	pid_data->previous_error = 0;
	pid_data->total_error = 0;

	pid_data->Kp = kp_init;
	pid_data->Ki = ki_init;
	pid_data->Kd = kd_init;

	pid_data->anti_windup_limit = anti_windup_limit_init;
}

void pid_reset(pid_str *pid_data)
{
	pid_data->total_error = 0;
	pid_data->previous_error = 0;
}

int pid_calculate(pid_str *pid_data, int setpoint, float process_variable)
{
	int error;
	float p_term, i_term, d_term;

	error = setpoint - process_variable;
	pid_data->total_error += error;

	if(pid_data->total_error > PID_MAX_INTEGRAL)
	{
		pid_data->total_error = PID_MAX_INTEGRAL;
	}
	if(pid_data->total_error < -PID_MAX_INTEGRAL)
	{
		pid_data->total_error = -PID_MAX_INTEGRAL;
	}

	p_term = (float)(pid_data->Kp * error);
	i_term = (float)(pid_data->Ki * pid_data->total_error) / SAMPLING_RATE;
	d_term = (float)(pid_data->Kd * SAMPLING_RATE * (error - pid_data->previous_error));

	pid_data->previous_error = error;

	return (int)(p_term + i_term + d_term);
}
