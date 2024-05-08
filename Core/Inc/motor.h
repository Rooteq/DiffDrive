#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_A_Kp					0.8
#define MOTOR_A_Ki					0.1
#define MOTOR_A_Kd					0.06
#define MOTOR_A_ANTI_WINDUP			1000

#define RPM_TO_RAD 0.10472
#define RAD_TO_DEG 57.29578
//#define MOTOR_A_Ki					0.6
//#define MOTOR_A_Kd					0.8
//#define MOTOR_A_ANTI_WINDUP			1000

#include <stdint.h>
#include <main.h>
#include "pid.h"
#include <stdbool.h>

typedef enum
{
	FORWARDS = 0,
	BACKWARDS = 1,
	STOP = 2
}MotorDirection;

typedef enum
{
	LEFT = 0,
	RIGHT = 1
}MotorSide;

// TODO: Sample the encoders every miliseconds from sysTick!!!

typedef struct{
	int16_t velocity;
	uint32_t lastCounterValue;
	TIM_HandleTypeDef *encoderHtim;
	// channel

}EncoderInstance; // change encoder for motor?


typedef struct
{
	MotorSide side;

	float rpm;
	float setRpm;
	int64_t position;

	int currentPWM;

	pid_str pid_controller;
	EncoderInstance encoder;
	TIM_HandleTypeDef *motorHtim;
	uint8_t motorChannel;

	bool stop;
}MotorInstance;

void motorUpdateVelocity(MotorInstance* motor);
void initMotor(MotorInstance* motor, TIM_HandleTypeDef *motorHtim, uint8_t motorChannel, TIM_HandleTypeDef *encoderHtim, MotorSide side);
void motorRegulateVelocity(MotorInstance* motor);
void motorSetSpeed(MotorInstance* motor, float setRpm);

void motorContinousSetSpeed(MotorInstance* motor, float setRpm);

void motorSetDirection(MotorInstance* motor, MotorDirection direction);

#endif /* INC_MOTOR_H_ */
