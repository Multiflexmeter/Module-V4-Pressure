#include "main.h"
#include "adc.h"

uint32_t ADC_Buffer[2];
uint16_t supplyVSENSORSLOT;
uint16_t supply3V3;

void ADC_Start(ADC_HandleTypeDef* hadc)
{
  HAL_ADC_Start_DMA(hadc, ADC_Buffer, 2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  supply3V3 = ADC_Buffer[0] * CONV_FACTOR_ADC;
  supplyVSENSORSLOT = ADC_Buffer[1] * CONV_FACTOR_ADC;
}
