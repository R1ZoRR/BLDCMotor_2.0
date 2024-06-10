/*
 * protocol.h
 *
 *  Created on: 6 июн. 2024 г.
 *      Author: warev_t8haga8
 */
#include "main.h"

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

uint16_t calculateCRC(uint8_t *message, uint16_t length);
void protocolParser();
void listening ();

#endif /* INC_PROTOCOL_H_ */
