#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include <main.h>

// TODO: Sample the encoders every miliseconds from sysTick!!!

typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t lastCounterValue;

	float w;
	float alfa;

}encoderInstance;

void updateEncoder(encoderInstance* encoderValue, TIM_HandleTypeDef *htim);
void resetEncoder(encoderInstance* encoderValue);

#endif /* INC_MOTOR_H_ */
