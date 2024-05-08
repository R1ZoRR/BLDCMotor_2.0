/*
 * MotorControl.c
 *
 *  Created on: 12 апр. 2024 г.
 *      Author: VSKoval
 */
#include "main.h"
#include "MotorControl.h"
#include "stdbool.h"

#define M_PI 				  3.14159265358979323846
#define TIME_INTERVAL_MS 	  50
#define NUMBER_OF_POLE 		  4
#define GEAR_RATIO	   		  28


const uint8_t commutationTable[6] = {0b100, 0b101, 0b001, 0b011, 0b010, 0b110};

uint8_t step = 0;

float U_PWM, V_PWM, W_PWM; // ШИМ на обмотках

uint16_t pwm = 3751;//2650;	//для тестов //2600 - порог минимальной скорости вращения? Потребление 0.14А

bool START_FLAG = 0;

//Для определения и корректировки скорости вращения
float K = 3.5;
float Kp;// = 1.1;// = 0.6 * K;
float Ki;// = 0.4;// = (2 * Kp)/50;
float Kd;// = 0.6;// = (Kp * 50)/8;

float targetRPM = 30;
float currentSpeed = 0.0;     // текущая скорость в RPM
float error, lastError;
float integral, derivative;

float time_50_ms = 0;

void initialize_PID_constants() {
	Kp = 0.6 * K;
	Ki = (2 * Kp) / TIME_INTERVAL_MS;
    Kd = (Kp * TIME_INTERVAL_MS) / 8;
}

///
uint16_t cnt_hall = 0;		//счетчик сработанного датчика Холла
uint16_t cnt_hall_last = 0;

/////////////////////////////////

void coil_AC(uint16_t pwm) { //1
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	TIM1->CCR1 = pwm/2;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	TIM8->CCR1 = pwm;
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
}
void coil_AB(uint16_t pwm) { //2
	TIM1->CCR1 = pwm;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	TIM1->CCR2 = pwm/2;
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
}
void coil_CB(uint16_t pwm) { //3
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

	TIM1->CCR2 = pwm;
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	TIM8->CCR1 = pwm/2;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}
void coil_CA(uint16_t pwm) { //4
	TIM1->CCR1 = pwm/2;
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	TIM8->CCR1 = pwm;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}
void coil_BA(uint16_t pwm) { //5
	TIM1->CCR1 = pwm;
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	TIM1->CCR2 = pwm/2;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
}
void coil_BC(uint16_t pwm) { //6
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

	TIM1->CCR2 = pwm;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	TIM8->CCR1 = pwm/2;
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
}

void motor_control(uint8_t command, uint16_t pwm) {
	switch (command) {
		case Eright:
			switch (step) {
			case 0b100:
				coil_BC(pwm); 	// A+1 B-0.5 C0
				break;
			case 0b101:
				coil_BA(pwm);	// A+0.5 B0 C-1
				break;
			case 0b001:
				coil_CA(pwm); 	// A0 B+1 C-0.5
				break;
			case 0b011:
				coil_CB(pwm);	// A-1 B+0.5 C0
				break;
			case 0b010:
				coil_AB(pwm); 	// A-0.5 B0 C+1
				break;
			case 0b110:
				coil_AC(pwm); 	// A0 B-1 C+0.5
				break;
			default:
				break;
			}
			break;

		case Estop:
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
			break;
//		case Eleft: 		//НЕ РАБОТАЕТ. НАДО ПОДУМАТЬ КАКИМ ОБРАЗОМ СДЕЛАТЬ
//			switch (step) {
//			case 0b100:
//				coil_BC(pwm); 	// A+1 B-0.5 C0
//				break;
//			case 0b101:
//				coil_BA(pwm);	// A+0.5 B0 C-1
//				break;
//			case 0b001:
//				coil_CA(pwm); 	// A0 B+1 C-0.5
//				break;
//			case 0b011:
//				coil_CB(pwm);	// A-1 B+0.5 C0
//				break;
//			case 0b010:
//				coil_AB(pwm); 	// A-0.5 B0 C+1
//				break;
//			case 0b110:
//				coil_AC(pwm); 	// A0 B-1 C+0.5
//				break;
//			default:
//				break;
//			}
//
//			break;

		default:
			break;
	}
}

void cur_sector() {
	hal_U = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
	hal_V = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	hal_W = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

	step = (hal_W) | (hal_V << 1) | (hal_U << 2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
	case GPIO_PIN_5:
	case GPIO_PIN_0:
	case GPIO_PIN_1:
		if (START_FLAG!=0){
			cnt_hall += 1;
			cur_sector();
			motor_control(Eright, pwm);
		}
		break;
	}
}

#define PULSES_PER_REVOLUTION 	6
#define GEAR_RATIO 				28 // Передаточное число
#define STEPS_REDUCER 			2
#define REDUCER_LENGTH 			47.4 // Длина ротора?

void calculateSpeed() {
//	currentSpeed = (cnt_hall * 60.0 * GEAR_RATIO) / (PULSES_PER_REVOLUTION * REDUCER_LENGTH); // Текущая скорость в об/мин
//	currentSpeed = (cnt_hall * 60.0 * REDUCER_LENGTH) / (PULSES_PER_REVOLUTION * GEAR_RATIO);
	currentSpeed = cnt_hall * 3.571428; // Текущая скорость в об/мин. Расчет: 60 c / 16.8 = 3.571428, где  336 * 0.05 = 16.8 - кол-во прерываний датчика холла за 0.05 с, для вращения со скоростью 1 об/с
	cnt_hall = 0; // Сброс счетчика импульсов
}

void calculatePID() {
    error = targetRPM - currentSpeed;
    integral += error;
    derivative = error - lastError;

    pwm = Kp*error + Ki*integral + Kd*derivative;
    if (pwm > 3751) pwm = 3751;
    if (pwm < 0) pwm = 2700;
    lastError = error;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {


	if (htim == &htim14) {
		time_50_ms += 0.5; // 0.5 мс - период таймера 14. Расчет: 1/ (TIM_FREQ / (TIM_ARR * TIM_PSC)) = 1 / (60 000 000 / ((499+1) * (59+1))) = 1/2000 = 0.5 мс
//		cur_sector();
		if (time_50_ms >= TIME_INTERVAL_MS) {
			calculateSpeed();
			calculatePID();
			time_50_ms = 0;
		}
//		motor_control(Eright, pwm);


	}
}

void loop(){
//	if(pwm < 3700) {
//		pwm += 2;
//		HAL_Delay(100);
//	}
}

void start(){
//	coil_BA(pwm);
//	HAL_Delay(100);
//	coil_CA(pwm);
//	HAL_Delay(100);
//	cnt_hall=0;
	START_FLAG=1;
	HAL_TIM_Base_Start_IT(&htim14);
	coil_BA(pwm);
	HAL_Delay(100);
	coil_CA(pwm);
	HAL_Delay(100);

}