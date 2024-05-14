/*
 * comms.c
 *
 *  Created on: Apr 15, 2024
 *      Author: Rooteq
 */

#include "comms.h"

void initPollTimers(PollTimers* timers) // TODO: move it to robot file
{
	timers->lastTx = HAL_GetTick();
	timers->lastPathPlan = HAL_GetTick();
	timers->lastRadarPoll = HAL_GetTick();
}

void initRxComms(RxCommsData* rxCommsData)
{
	rxCommsData->handleIncomingData = 0;
	rxCommsData->dataSize = 0;
}

void initTxComms(TxCommsData* txCommsData, UART_HandleTypeDef *huart, Position* pos, StateFlag *flag)
{
	txCommsData->huart = huart;
	txCommsData->flag = flag;
	txCommsData->pos = pos;
}

void int16_to_bytes(int16_t value, uint8_t *buffer) {
    buffer[1] = (uint8_t)(value & 0xFF);
    buffer[0] = (uint8_t)((value >> 8) & 0xFF);
}

uint16_t crc16(const uint8_t* data_p, uint8_t length){
    uint8_t x;
    uint16_t crc = 0x1D0F;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

void UARTSendPos(TxCommsData* txCommsData)
{
	static uint8_t tx_buffer[POS_BUFFER_SIZE];

    tx_buffer[0] = START_BYTE;

    tx_buffer[1] = *(txCommsData->flag);

    int16_to_bytes((int16_t)txCommsData->pos->x, &tx_buffer[2]);
    int16_to_bytes((int16_t)txCommsData->pos->y, &tx_buffer[4]);
    int16_to_bytes((int16_t)(RAD_TO_DEG * txCommsData->pos->ang), &tx_buffer[6]);

    uint16_t crc = crc16(&tx_buffer[1], 7);
    tx_buffer[9] = (uint8_t)(crc & 0xFF);
    tx_buffer[8] = (uint8_t)((crc >> 8) & 0xFF);

    tx_buffer[10] = END_BYTE;
    HAL_UART_Transmit_IT(txCommsData->huart, tx_buffer, sizeof(tx_buffer));
}

void handleCommand(RxCommsData* rxCommsData, Robot* robot)
{
	robot->goToPoint = false;
	switch(rxCommsData->MainBuf[1])
	{
	case 'F':
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		setStateFlag(robot, MANUAL_MOVE);
		motorSetConstSpeed(&(robot->motorRight), 12);
		motorSetConstSpeed(&(robot->motorLeft), 12);
		break;
	case 'B':
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		setStateFlag(robot, MANUAL_MOVE);
		motorSetConstSpeed(&(robot->motorRight), -12);
		motorSetConstSpeed(&(robot->motorLeft), -12);
		break;
	case 'L':
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		setStateFlag(robot, MANUAL_MOVE);
		motorSetConstSpeed(&(robot->motorRight), 10);
		motorSetConstSpeed(&(robot->motorLeft), -10);
		break;
	case 'R':
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		setStateFlag(robot, MANUAL_MOVE);
		motorSetConstSpeed(&(robot->motorRight), -10);
		motorSetConstSpeed(&(robot->motorLeft), 10);
		break;
	case 'S':
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		setStateFlag(robot, STOPPED);
		motorSetConstSpeed(&(robot->motorRight), 0);
		motorSetConstSpeed(&(robot->motorLeft), 0);
		break;
	default:
		break;
	}
}

void handleDestinationCommand(RxCommsData* rxCommsData, Robot* robot)
{
	if(crc16(&(rxCommsData->MainBuf[1]), 6) == 0)
	{
	    int16_t tmpX = (int16_t)((rxCommsData->MainBuf[1] << 8) | rxCommsData->MainBuf[2]);
	    int16_t tmpY = (int16_t)((rxCommsData->MainBuf[3] << 8) | rxCommsData->MainBuf[4]);

	    robot->destination.xd = tmpX;
	    robot->destination.yd = tmpY;

	    beginPositionControl(robot, tmpX, tmpY);
	}
}

void handleRx(RxCommsData* rxCommsData, Robot* robot) // pass internal state as argument so that it could do something xdd
{
	if(rxCommsData->handleIncomingData == 1)
	{
		if(rxCommsData->dataSize == 2)
		{
			handleCommand(rxCommsData, robot);
		}
		else if(rxCommsData->dataSize == 8) // {p11|p12|p21|p22|crc1|crc2|}
		{
			handleDestinationCommand(rxCommsData, robot);
		}

		rxCommsData->handleIncomingData = 0;
	}
}

void handleTx(TxCommsData* txCommsData, PollTimers* pollTimers)
{
	if(HAL_GetTick() - pollTimers->lastTx > 1000)
	{
		UARTSendPos(txCommsData);
		pollTimers->lastTx = HAL_GetTick();
	}
}

