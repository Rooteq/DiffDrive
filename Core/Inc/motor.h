#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_A_Kp					1.2
#define MOTOR_A_Ki					0.06
#define MOTOR_A_Kd					0.4
#define MOTOR_A_ANTI_WINDUP			1000

#include <stdint.h>
#include <main.h>
#include "pid.h"

typedef enum
{
	CW = 0,
	CCW = 1
}motorDirection;

// TODO: Sample the encoders every miliseconds from sysTick!!!

typedef struct{
	int16_t velocity;
	uint32_t lastCounterValue;
	TIM_HandleTypeDef *encoderHtim;
	// channel

}EncoderInstance; // change encoder for motor?


typedef struct
{

	float rpm;
	float setRpm;
	int64_t position;

	uint16_t currentPWM;
	motorDirection direction;

	pid_str pid_controller;
	EncoderInstance encoder;
	TIM_HandleTypeDef *motorHtim;
	uint8_t motorChannel;
}MotorInstance;

void motorUpdateVelocity(MotorInstance* motor);
void initMotor(MotorInstance* motor, TIM_HandleTypeDef *motorHtim, uint8_t motorChannel, TIM_HandleTypeDef *encoderHtim);
void motorRegulateVelocity(MotorInstance* motor);
void motorSetSpeed(MotorInstance* motor, float setRpm);

#endif /* INC_MOTOR_H_ */
