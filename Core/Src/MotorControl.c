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
#define NUMBER_OF_POLE 		  4
#define GEAR_RATIO	   		  28

extern ADC_HandleTypeDef hadc1;

const uint8_t commutationTable[6] = {0b100, 0b101, 0b001, 0b011, 0b010, 0b110};

uint8_t step = 0;

float U_PWM, V_PWM, W_PWM; // ШИМ на обмотках

uint16_t pwm = 2600; //2650 - для тестов; 2600 - порог минимальной скорости вращения? Потребление 0.14А; 3750 - Максимальный ШИМ. Огромный ток.

bool START_FLAG = 0;

//Для определения и корректировки скорости вращения
float K = 3.5;
float Kp;// = 1.1;// = 0.6 * K;
float Ki;// = 0.4;// = (2 * Kp)/50;
float Kd;// = 0.6;// = (Kp * 50)/8;
float TIME_INTERVAL_MS = 100; // Интервал расчета ПИД
float TIME_PWM_MS = 2;

float targetRPM = 60;
float currentSpeed = 0;     // текущая скорость в RPM
float error, lastError;
float integral, derivative;

float pid_time = 0;

float I_a = 0;
float I_b = 0;
float I_c = 0;
float I_alpha = 0;
float I_beta = 0;
float I_d = 0;
float I_q = 0;
float summ = 0;

void initialize_PID_constants() {
	Kp = 30;
	Ki = (2 * Kp) / TIME_INTERVAL_MS;
    Kd = (Kp * TIME_INTERVAL_MS) / 8;
}

///
uint16_t cnt_hall = 0;		//счетчик сработанного датчика Холла
uint16_t cnt_hall_last = 0;

/////////////////////////////////

// Функция изменяет ШИМ в соответсвии с указанным вектором тяги
void move_rotor(float to_angle) {
	// Расчет потенциалов и заполнения шима для фаз
	U_PWM = pwm*(sin((to_angle)     * M_PI/180) + sin((to_angle)     * M_PI/60)/4);
	V_PWM = pwm*(sin((to_angle+120) * M_PI/180) + sin((to_angle+120) * M_PI/60)/4);
	W_PWM = pwm*(sin((to_angle+240) * M_PI/180) + sin((to_angle+240) * M_PI/60)/4);

	/////////////////////////////////
	// Перенастройка шима на фазах

	if(U_PWM >= 0) { // if, т. к. позитивными и негативными ключами управляют разные каналы таймеров
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = (uint16_t)U_PWM;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	}
	else {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		TIM1->CCR1 = (uint16_t)(-U_PWM);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	}

	if(V_PWM >= 0) {
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		TIM1->CCR2 = (uint16_t)V_PWM;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	}
	else {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		TIM1->CCR2 = (uint16_t) (-V_PWM);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	}

	if(W_PWM >= 0) {
		HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
		TIM8->CCR1 = (uint16_t)W_PWM;
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	}
	else {
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
		TIM8->CCR1 = (uint16_t) (-W_PWM);
		HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	}

	// Сброс счетчиков таймеров для синхронизации
	TIM1->CNT = 0;
	TIM8->CNT = 0;
	return;
}

// Функция, управляющая последовательностью переключением обмоток двигателя
float offset = -180; // Датчики развернуты на +26.5 градусов отностительно статора; сдвиг в меньше 27- движение по часовой, в больше - против часовой
void motor_control(uint8_t command, uint16_t pwm) {
	switch (command) {
		case Eright:
			switch (step) {
			case 0b101: // 5
//				move_rotor(0 + offset);
				move_rotor(60 + offset);
				break;
			case 0b001: // 1
//				move_rotor(60 + offset);
				move_rotor(60 + offset);
				break;
			case 0b011: // 3
//				move_rotor(120 + offset);
				move_rotor(60 + offset);

				break;
			case 0b010: // 2
				move_rotor(-60 + offset);
				break;
			case 0b110: // 6
				move_rotor(-60 + offset);
				break;
			case 0b100: // 4
//				move_rotor(300 + offset);
				move_rotor(-60 + offset);
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

// Чтение показателей датчиков Холла
void cur_sector() {
	hal_U = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5);
	hal_V = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	hal_W = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
	step = (hal_W) | (hal_V << 1) | (hal_U << 2);
}

float currentAngle = 35;
float rememberAngle = 0;
// Обработчик прерываний датчиков Холла
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
	case GPIO_PIN_5:
	case GPIO_PIN_0:
	case GPIO_PIN_1:
		if (START_FLAG!=0){
			cnt_hall += 1;
			cur_sector();
			motor_control(Eright, pwm);
			I_alpha = I_a + I_b * (-1/2) + I_c * (-1/2);
			I_beta = I_b * sqrt(3)/2 + I_c * (-sqrt(3)/2);
			I_d = 2/3*(cos(currentAngle  * M_PI/180)+sin(currentAngle * M_PI/180));
			I_q = 2/3*(-sin(currentAngle * M_PI/180)+cos(currentAngle * M_PI/180));
		}
		break;
	}
}

#define PULSES_PER_REVOLUTION 	6
#define GEAR_RATIO 				28 // Передаточное число
#define STEPS_REDUCER 			2

void calculateSpeed() {
	currentSpeed = cnt_hall * 60 / (336 * pid_time / 1000); // Текущая скорость в об/мин. Расчет: cnt_hall * 3.571428 = 60 c * cnt_hall / 16.8 = , где  16.8 = 336 * 0.05 - кол-во прерываний датчика холла за 0.05 с, для вращения со скоростью 1 об/с
	cnt_hall = 0; // Сброс счетчика импульсов
}

void calculatePID() {
    error = targetRPM - currentSpeed;
    integral += error;
    derivative = error - lastError;

    float tmp_pwm = Kp*error + Ki*integral + Kd*derivative;
    if ((tmp_pwm - pwm) > 0) {
    	if ((tmp_pwm - pwm) > 50) {
    		pwm += 50;
    	}
    	else {
    		pwm += tmp_pwm - pwm;
    	}
    }
    else {
		if ((tmp_pwm - pwm) < -50) {
			pwm -= 50;
		}
		else {
			pwm += tmp_pwm - pwm;
		}
    }
    if (pwm > 3751) pwm = 3751;
    if (pwm < 0) pwm = 2700;
    lastError = error;
}

// Для АЦП
uint16_t rawValues[3];


float TIME_PWM = 0;
float cnt_hall_max = 0;
float offset_increment = 0.002;
// Обработчик прерываний таймера
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == &htim14) {
		pid_time += 0.5; // 0.5 мс - период таймера 14. Расчет: (TIM_ARR * TIM_PSC) / TIM_FREQ / ) = ((499+1) * (59+1)) / 60 000 000 = 0,0005 с = 0.5 мс
		TIME_PWM += 0.5;

		if (cnt_hall >= 2000) { // (pid_time >= TIME_INTERVAL_MS)||(cnt_hall >= 500)
			cnt_hall_max = cnt_hall;
			calculateSpeed();
//			calculatePID();
			pid_time = 0;
			offset += offset_increment;

		}

		if (TIME_PWM >= TIME_PWM_MS) {
//			currentAngle += 5;// targetRPM * 360 / (60 * 20)
			currentAngle = fmodf(currentAngle, 360);

			motor_control(Eright, pwm);
			TIME_PWM = 0;
		}



	}
}

// Обработчик прерываний АЦП
float Sensitivity_I = 0.0066; // Чувствительность датчика тока в мВ/мА

// Обработка показаний АЦП по готовности всех 3-х каналов
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// где rawValues - показания АЦП; 3.3 В - питание АПЦ; 4096 - количество уровней квантования при 12 разрядах; ((5.62+10)/10)) - компенсация нормирующего делителя напряжения; 2.5 В - компинсация кривой датчика тока (точка отсчета 0 мА);
	// Расчет: (((2000*3.3)/4096)*(5.62+10)/10-2.5)/0.0066 = 2,56 мА; (((2000*3.3)/4096)*(5.62+10)/10-2.5)/0.0066-6,9739139441
	I_a = ((rawValues[0] * 3.3 / 4096) * (5.62+10) / 10 - 2.5) / Sensitivity_I;
	I_b = ((rawValues[1] * 3.3 / 4096) * (5.62+10) / 10 - 2.5) / Sensitivity_I;
	I_c = ((rawValues[2] * 3.3 / 4096) * (5.62+10) / 10 - 2.5) / Sensitivity_I;
	summ = I_a + I_b + I_c;
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

// функция - инициализация
void start(){
	initialize_PID_constants();
	START_FLAG=1;
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)rawValues, 3);
	cur_sector();
//	move_rotor(0);
//	motor_control(Eright, pwm);
}

// Бесконечный цикл
void loop(){

}
