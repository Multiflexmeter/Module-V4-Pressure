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
uint16_t supplyVSENSORSLOT;

void ADC_Init(ADC_HandleTypeDef *hadc)
{
  hadc->Instance = ADC1;
  hadc->Init.OversamplingMode = DISABLE;
  hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  hadc->Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ContinuousConvMode = ENABLE;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.DMAContinuousRequests = DISABLE;
  hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.LowPowerFrequencyMode = ENABLE;
  hadc->Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CH6(ADC_HandleTypeDef *hadc)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Configure for the selected ADC regular channel its corresponding rank in the sequencer*/
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CH7(ADC_HandleTypeDef *hadc)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Configure for the selected ADC regular channel its corresponding rank in the sequencer*/
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

uint16_t ADC_Vsensor_Measure(ADC_HandleTypeDef *hadc)
{
  uint16_t value = 0;

  ADC_Init(hadc);
  ADC_Select_CH6(hadc);
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 1000);
  value = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  HAL_ADC_DeInit(hadc);

  return value * CONV_FACTOR_ADC;
}

uint16_t ADC_Vsensorslot_Measure(ADC_HandleTypeDef *hadc)
{
  uint16_t value = 0;

  ADC_Init(hadc);
  ADC_Select_CH7(hadc);
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 1000);
  value = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  HAL_ADC_DeInit(hadc);

  return value * CONV_FACTOR_ADC;
}
