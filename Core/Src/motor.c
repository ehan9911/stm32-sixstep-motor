/*
 * motor.c
 *
 *  Created on: Apr 16, 2021
 *      Author: 86177
 */

#include "motor.h"
#include "main.h"
#include <stdio.h>
#include "stm32f3xx_ll_tim.h"

static unsigned short bldcm_pluse = 0;
static BLDCM_DATA_T bldcm_data;
static unsigned short update = 0;
static TIM_HandleTypeDef *motorHtim;



MOTOR_DIR_E GetBldcmDirection(void)
{
	return bldcm_data.direction;
}

void SetBldcmDirection(MOTOR_DIR_E dir)
{
	bldcm_data.direction = dir;
	printf("set motor dir = %d\r\n", dir);
}

void SetBldcmPeriod(uint32_t period)
{
	bldcm_data.period = period;
}

void SetBldcmPwmMode(uint16_t pwmMode)
{
	bldcm_data.pwmMode = pwmMode;
	printf("pwmMode=%04x, pwm1=%04x, pwm2=%04x\r\n", pwmMode, TIM_OCMODE_PWM1, TIM_OCMODE_PWM2);
}



void BldcmSetSpeed(uint16_t pluse)
{
	bldcm_pluse = pluse;
	printf("init pluse = %d\r\n", pluse);
}

int BldcmIncrSpeed(void)
{
	bldcm_pluse += bldcm_data.period / 20;
	if (bldcm_pluse > bldcm_data.period * 0.3) {
		bldcm_pluse = bldcm_data.period * 0.3;
		printf("pluse = %d, next act: dec\r\n", bldcm_pluse);
		return 1;
	}
	printf("motor speed incr, pluse = %d\r\n", bldcm_pluse);
	return 0;
}

int BldcmDecSpeed(void)
{
	unsigned short step = bldcm_data.period / 20;

	if (bldcm_pluse > bldcm_data.period * 0.1) {
		bldcm_pluse -= step;
	} else {
		bldcm_pluse = bldcm_data.period * 0.1;
		printf("pluse = %d, next act: add\r\n", bldcm_pluse);
		return 1;
	}
	printf("motor speed dec, pluse = %d\r\n", bldcm_pluse);
	return 0;
}

uint8_t GetHallState(void)
{
	uint8_t state = 0;
	if (HAL_GPIO_ReadPin(Hall_U_GPIO_Port, Hall_U_Pin) != GPIO_PIN_RESET) { //PA6
		state |= 1;
	}
	if (HAL_GPIO_ReadPin(Hall_V_GPIO_Port, Hall_V_Pin) != GPIO_PIN_RESET) { //PB5
		state |= 2;
	}
	if (HAL_GPIO_ReadPin(Hall_W_GPIO_Port, Hall_W_Pin) != GPIO_PIN_RESET) { //PC8
		state |= 4;
	}
	return state;
}



void MotorRun(TIM_HandleTypeDef *htimx, uint8_t step)
{
	unsigned short pluse = bldcm_pluse;
	if (bldcm_data.pwmMode == TIM_OCMODE_PWM2) {
		pluse = bldcm_data.period - bldcm_pluse;
	}

    switch(step)
	{
	    case 1: //C+ B-  1, 3N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N);
	   break;

	   case 2: //B+       2, 1N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH1N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
		break;

		case 3: //C+ A-  2, 3N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N);
		break;

		case 4: //A+ C- 3->2N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH2N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3N);
		break;

		case 5: //A+ B-  1->2N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3N);
		break;

		case 6: //B+ C-  3->1N
			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, 0);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, pluse);
			HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_SET);
			LL_TIM_CC_EnableChannel(htimx->Instance, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
		break;

		default:  //关闭输出
			HAL_GPIO_WritePin(MOTOR_1_OCPWM1_GPIO_PORT, MOTOR_1_OCPWM1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_1_OCPWM2_GPIO_PORT, MOTOR_1_OCPWM2_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_1_OCPWM3_GPIO_PORT, MOTOR_1_OCPWM3_PIN, GPIO_PIN_RESET);
			LL_TIM_CC_DisableChannel(htimx->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 |
			                        			      LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
		break;
	}
    HAL_TIM_GenerateEvent(htimx, TIM_EVENTSOURCE_COM);
}

uint8_t PWM_GetRevStep(uint8_t step)
{
	switch(step) {
		case 1:
			return 6;
		case 2:
			return 5;
		case 3:
			return 4;
		case 4:
			return 3;
		case 5:
			return 2;
		case 6:
			return 1;
		default:
			return 0;
	}
	return 0;
}

void MotorAProc(void)
{
    uint8_t step = GetHallState();

    static uint8_t prestep = 0;
    
	TIM_HandleTypeDef *htimx = motorHtim;
	if (htimx == 0) {
		printf("motor A timer not init\r\n");
		return;
	}
	//printf("step = %d\r\n", step);
    if (step == prestep) {
    	return;
    }
    prestep = step;


    if (GetBldcmDirection() == MOTOR_REV) {
    	step = PWM_GetRevStep(step);
    }
    MotorRun(htimx, step);
    HAL_TIM_GenerateEvent(htimx, TIM_EVENTSOURCE_COM);
}


void MotorAInit(TIM_HandleTypeDef *htimx, ADC_HandleTypeDef *hadc)
{
	motorHtim = htimx;
  	HAL_ADC_Start_IT(hadc);
}




void HallEnable(TIM_HandleTypeDef *htimMaster, TIM_HandleTypeDef *htimSlave)
{
	__HAL_TIM_ENABLE_IT(htimMaster, TIM_IT_TRIGGER);
	__HAL_TIM_ENABLE_IT(htimMaster, TIM_IT_UPDATE);
	HAL_TIMEx_HallSensor_Start(htimMaster);
	HAL_TIM_TriggerCallback(htimSlave);
}

void HallDisable(TIM_HandleTypeDef *htimx)
{
	__HAL_TIM_DISABLE_IT(htimx, TIM_IT_TRIGGER);
	__HAL_TIM_DISABLE_IT(htimx, TIM_IT_UPDATE);
	HAL_TIMEx_HallSensor_Stop(htimx);
}


void MotorStop(TIM_HandleTypeDef *htimx)
{
	__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, 0);

	HAL_GPIO_WritePin(MOTOR_1_OCNPWM1_GPIO_PORT, MOTOR_1_OCNPWM1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_1_OCNPWM2_GPIO_PORT, MOTOR_1_OCNPWM2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_1_OCNPWM3_GPIO_PORT, MOTOR_1_OCNPWM3_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htimx)
{
	printf("safty check\r\n");
	if (update++ > 1) {
		printf("timeout\r\n");
		update = 0;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
	HallDisable(htimx);
	MotorStop(htimx);
}

void BldcmEnable(TIM_HandleTypeDef *htimMaster, TIM_HandleTypeDef *htimSlave)
{
	HallEnable(htimMaster, htimSlave);
}


void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim)
{
	printf("%s %d\r\n", __func__, __LINE__);
}


void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
	printf("%s %d\r\n", __func__, __LINE__);
}

void MotorCfg(void)
{
	MOTOR_DIR_E dir = MOTOR_FWD;
	static uint8_t dir_set = 0;
	static uint8_t speed_set = 0;
	static uint8_t speed_add = 1;

	// 电流�?�?, PC1管脚

	if (HAL_GPIO_ReadPin(Motor_Dir_GPIO_Port, Motor_Dir_Pin) == GPIO_PIN_SET) {
		dir_set = 1;
	}
	if ((dir_set == 1) && (HAL_GPIO_ReadPin(Motor_Dir_GPIO_Port, Motor_Dir_Pin) == GPIO_PIN_RESET)) {
		dir_set = 0;
		dir = GetBldcmDirection();
		if (dir == MOTOR_FWD) {
			dir = MOTOR_REV;
		} else {
			dir = MOTOR_FWD;
		}
		SetBldcmDirection(dir);
	}

	if (HAL_GPIO_ReadPin(Motor_SpeedAdj_GPIO_Port, Motor_SpeedAdj_Pin) == GPIO_PIN_SET) {
		speed_set = 1;
	}
	if ((speed_set == 1) && (HAL_GPIO_ReadPin(Motor_SpeedAdj_GPIO_Port, Motor_SpeedAdj_Pin) == GPIO_PIN_RESET)) {
		speed_set = 0;
		if (speed_add) {
			if (BldcmIncrSpeed()) {
				speed_add = 0;
			}
		} else {
			if (BldcmDecSpeed()) {
				speed_add = 1;
			}
		}
	}
}



