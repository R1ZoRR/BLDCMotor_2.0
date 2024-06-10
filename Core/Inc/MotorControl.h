/*
 * MotorControl.h
 *
 *  Created on: 12 апр. 2024 г.
 *      Author: VSKoval
 */
#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef SRC_MOTORCONTROL_H_
#define SRC_MOTORCONTROL_H_

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim14;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern uint16_t receiveCounter;
/* Щеточный, щеточный мотор... Не баловство...
void forward();
void reverse();
void motor_control();
*/

enum move_control {
	Estop,
	Eleft,
	Eright
};

GPIO_PinState hal_U,hal_V,hal_W;
uint8_t lastMode;

void motor_control(uint8_t modeControl, uint16_t pwm);
void loop();
void start();

#endif /* SRC_MOTORCONTROL_H_ */
