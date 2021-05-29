/*
 * motor.h
 *
 *  Created on: Apr 16, 2021
 *      Author: 86177
 */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"
#include "stm32f3xx_hal_gpio.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MOTOR_2_OCPWM1_GPIO_PORT   GPIOC
#define MOTOR_2_OCPWM1_PIN         GPIO_PIN_6
#define MOTOR_2_OCPWM2_GPIO_PORT   GPIOC
#define MOTOR_2_OCPWM2_PIN         GPIO_PIN_7
#define MOTOR_2_OCPWM3_GPIO_PORT   GPIOB
#define MOTOR_2_OCPWM3_PIN         GPIO_PIN_9

#define MOTOR_2_OCNPWM1_GPIO_PORT  GPIOC
#define MOTOR_2_OCNPWM1_PIN        GPIO_PIN_10
#define MOTOR_2_OCNPWM2_GPIO_PORT  GPIOC
#define MOTOR_2_OCNPWM2_PIN        GPIO_PIN_11
#define MOTOR_2_OCNPWM3_GPIO_PORT  GPIOC
#define MOTOR_2_OCNPWM3_PIN        GPIO_PIN_12

#define MOTOR_1_OCPWM1_GPIO_PORT   GPIOA
#define MOTOR_1_OCPWM1_PIN         GPIO_PIN_8
#define MOTOR_1_OCPWM2_GPIO_PORT   GPIOA
#define MOTOR_1_OCPWM2_PIN         GPIO_PIN_9
#define MOTOR_1_OCPWM3_GPIO_PORT   GPIOA
#define MOTOR_1_OCPWM3_PIN         GPIO_PIN_10

#define MOTOR_1_OCNPWM1_GPIO_PORT  GPIOA
#define MOTOR_1_OCNPWM1_PIN        GPIO_PIN_11
#define MOTOR_1_OCNPWM2_GPIO_PORT  GPIOA
#define MOTOR_1_OCNPWM2_PIN        GPIO_PIN_12
#define MOTOR_1_OCNPWM3_GPIO_PORT  GPIOB
#define MOTOR_1_OCNPWM3_PIN        GPIO_PIN_15


typedef enum {
	MOTOR_FWD = 0,
	MOTOR_REV = 1,
}MOTOR_DIR_E;

typedef struct {
	MOTOR_DIR_E direction;
	uint8_t     isEnable;
	uint16_t    dutyFactor;
	uint32_t    lockTimeout;
	uint32_t    period;
	uint32_t    pluse;
	uint16_t    pwmMode;
}BLDCM_DATA_T;

#define HALL_TIM_CLK_ENABLE()   __HAL_RCC_TIM3_CLK_ENABLE()

extern void BldcmEnable(TIM_HandleTypeDef *htimMaster, TIM_HandleTypeDef *htimSlave);
extern void BldcmSetSpeed(unsigned short pluse);
extern void motor_test(void);
extern uint8_t GetHallState(void);
extern void SetBldcmDirection(MOTOR_DIR_E dir);
extern MOTOR_DIR_E GetBldcmDirection(void);
extern void SetBldcmPeriod(uint32_t period);
extern void MotorCfg(void);
extern void MotorAInit(TIM_HandleTypeDef *htimx, ADC_HandleTypeDef *hadc);
extern void MotorAProc(void);
extern void SetBldcmPwmMode(uint16_t pwmMode);
#endif /* INC_MOTOR_H_ */
