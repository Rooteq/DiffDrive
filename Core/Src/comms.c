/*
 * comms.c
 *
 *  Created on: Apr 15, 2024
 *      Author: Rooteq
 */

#include "comms.h"

void initPollTimers(PollTimers* timers)
{
	timers->lastTx = HAL_GetTick();
}


void initRxComms(RxCommsData* rxCommsData)
{
	rxCommsData->handleIncomingData = 0;
	rxCommsData->dataSize = 0;
}

void initTxComms(TxCommsData* txCommsData, UART_HandleTypeDef *huart, Position* pos, ErrorFlag *flag)
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

    int16_to_bytes(txCommsData->pos->x, &tx_buffer[2]);
    int16_to_bytes(txCommsData->pos->y, &tx_buffer[4]);
    int16_to_bytes(txCommsData->pos->ang, &tx_buffer[6]);

    uint16_t crc = crc16(&tx_buffer[1], 7);
    tx_buffer[9] = (uint8_t)(crc & 0xFF);
    tx_buffer[8] = (uint8_t)((crc >> 8) & 0xFF);

    tx_buffer[10] = END_BYTE;
    HAL_UART_Transmit_IT(txCommsData->huart, tx_buffer, sizeof(tx_buffer)); // handle buffer overflow?
}

void handleCommand(RxCommsData* rxCommsData, int16_t* setVelocity)
{
	switch(rxCommsData->MainBuf[1])
	{
	case 'F':
		*setVelocity = 20;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		break;

	case 'S':
		*setVelocity = 0;
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}



void handleRx(RxCommsData* rxCommsData, int16_t* setVelocity) // pass internal state as argument so that it could do something xdd
{
	if(rxCommsData->handleIncomingData == 1)
	{
		if(rxCommsData->dataSize == 2) // change to more lolz - add crc etc
		{
			handleCommand(rxCommsData, setVelocity);
		}
//		else if(dataSize == ) // handle position
		rxCommsData->handleIncomingData = 0;
	}
}

void handleTx(TxCommsData* txCommsData, PollTimers* pollTimers)
{// make pollTimers internal? call it on interrupts?
	if(HAL_GetTick() - pollTimers->lastTx)
	{
		UARTSendPos(txCommsData);
		pollTimers->lastTx = HAL_GetTick();
	}
}

