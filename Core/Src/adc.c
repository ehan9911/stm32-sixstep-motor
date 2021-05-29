/*
 * adc.c
 *
 *  Created on: May 7, 2021
 *      Author: 86177
 */
#include "adc.h"
#include "main.h"
#include "motor.h"
#include <stdio.h>


void My_ADC_Init(ADC_HandleTypeDef *hadc)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc->Instance = ADC1;
  hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DMAContinuousRequests = DISABLE;
  hadc->Init.EOCSelection = DISABLE;
  hadc->Init.LowPowerAutoWait = DISABLE;
  //hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

unsigned short GetAdc(ADC_HandleTypeDef *adc)
{
	return HAL_ADC_GetValue(adc);
}
extern TIM_HandleTypeDef htim1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	unsigned short adc = HAL_ADC_GetValue(hadc);
	MotorAProc();
}


