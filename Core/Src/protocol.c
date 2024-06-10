/*
 * protocol.c
 *
 *  Created on: 6 июн. 2024 г.
 *      Author: warev_t8haga8
 */

#define PROTOCOL_BUFF_SIZE 1024
#define SLAVE_ID 1
#include "protocol.h"
#include "MotorControl.h"


uint8_t receivedData;
uint8_t receivedBuff[PROTOCOL_BUFF_SIZE];
uint8_t response[PROTOCOL_BUFF_SIZE];
uint16_t holdingRegisters[PROTOCOL_BUFF_SIZE];
uint8_t responseSize;


uint16_t receiveCounter = 0;
void protocolReceive(uint8_t data) {
	receivedBuff[receiveCounter] = data;
	receiveCounter++;
	if (receiveCounter > (PROTOCOL_BUFF_SIZE-1)) {
		receiveCounter = 0;
	}
}

uint16_t calculateCRC(uint8_t *message, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= message[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t crc;
void protocolParser() {
	// Проверка размера сообщения
	if(receiveCounter<2){
//		ui->txtError->setText("NO DATA");
		return;
	}

	// Проверка ID получателя
	if(receivedBuff[0] != (uint8_t)SLAVE_ID)
	{
//		ui->txtError->setText("BAD ADR");
		return;
	}

	// Проверка CRC
	crc = calculateCRC(&receivedBuff, receiveCounter-2);
	if( !((receivedBuff[receiveCounter-1] == ((crc >> 8) & (0xFF))) && (receivedBuff[receiveCounter-2] == ((crc) & (0xFF)))))
	{
//		ui->txtError->setText("BAD CRC");
		return;
	}

	// Обработка данных
	switch (receivedBuff[1]) {
	case 0x03:
	case 0x04:
		// Чтение нескольких регистров
		// первые 2 байта адрес первого регистра
		// 2 байта кол-во регистров
		HAL_UART_Transmit(&huart3, response, 8, (10/19200));
		break;
	case 0x10:
		// Запись данных
		// первые 2 байта адрес первого регистра
		// 2 байта кол-во регистров
		// 1 байт кол-во байт далее
		// Байты данных
		for (int i = 0; i>receivedBuff[6]; i+=2) {
			holdingRegisters[receivedBuff[3]+receivedBuff[2]<<8+i] = receivedBuff[8+i]+receivedBuff[7+i]<<8;
		}

		// Отправка ответа
		responseSize = 8;
		response[0] = SLAVE_ID;
		response[1] = 0x10;
		response[2] = receivedBuff[2];
		response[3] = receivedBuff[3];
		response[4] = receivedBuff[4];
		response[5] = receivedBuff[5];
		uint16_t crc = calculateCRC(&response, responseSize-2);
		response[6] = crc>>8;
		response[7] = crc & (0xFF);
		HAL_UART_Transmit(&huart3, response, responseSize, 10);
		return;
	default:
//		ui->txtError->setText("NO SUCH FUNC");
		return;
	}
}

void listening () {
//    HAL_UART_StateTypeDef sss =  huart3.gState;
	HAL_UART_Receive_IT(&huart3, &receivedData, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3) {
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_TIM_Base_Start_IT(&htim3);
		protocolReceive(receivedData); // Заполение буфера
		HAL_UART_Receive_IT(&huart3, &receivedData, 1);
	}
}
