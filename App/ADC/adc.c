/**
  ******************************************************************************
  * @file           adc.c
  * @brief          ADC functions
  * @author         D.Kerstens
  ******************************************************************************
  */

#include "main.h"
#include "adc.h"

uint16_t supplyVSENSOR;

uint16_t ADC_Vsensor_Measure(ADC_HandleTypeDef *hadc)
{
  uint16_t value = 0;

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 1000);
  value = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);

  return value * CONV_FACTOR_ADC;
}
