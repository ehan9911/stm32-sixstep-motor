/*
 * adc.h
 *
 *  Created on: May 7, 2021
 *      Author: 86177
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f3xx_hal.h"

extern unsigned short GetAdc(ADC_HandleTypeDef *adc);
extern void My_ADC_Init(ADC_HandleTypeDef *hadc);

#endif /* INC_ADC_H_ */
