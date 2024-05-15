/*
 * comms.h
 *
 *  Created on: Apr 15, 2024
 *      Author: Rooteq
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include <stdint.h>
#include <main.h>
#include "robot.h"

#define RxBuf_SIZE 10 // max message sizes
#define MainBuf_SIZE 20

#define POS_BUFFER_SIZE 12
#define START_BYTE 0xAA // Znacznik początku ramki
#define END_BYTE 0x55   // Znacznik końca ramki
#define CRC_POLYNOMIAL 0x1021 // Standard CRC-16-ANSI polynomial

typedef struct{
	uint8_t handleIncomingData;
	uint8_t dataSize;
	uint8_t RxBuf[RxBuf_SIZE];
	uint8_t MainBuf[MainBuf_SIZE];

} RxCommsData;

typedef struct{
	UART_HandleTypeDef *huart;
	Position* pos;
	StateFlag *flag;
	uint8_t *obstacleDistance;
} TxCommsData;


void initPollTimers(PollTimers* timers);

void initRxComms(RxCommsData* rxCommsData);
void initTxComms(TxCommsData* txCommsData, UART_HandleTypeDef *huart, Position* pos, StateFlag *flag, uint8_t *obstacleDistance);

void int16_to_bytes(int16_t value, uint8_t *buffer);
uint16_t crc16(const uint8_t* data_p, uint8_t length);
void UARTSendPos(TxCommsData* txCommsData);

void handleCommand(RxCommsData* rxCommsData, Robot* robot);
void handleDestinationCommand(RxCommsData* rxCommsData, Robot* robot);


void handleRx(RxCommsData* rxCommsData, Robot* robot);
void handleTx(TxCommsData* txCommsData, PollTimers* pollTimers); // make pollTimers internal? call it on interrupts?



#endif /* INC_COMMS_H_ */
