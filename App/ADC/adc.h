/**
  ******************************************************************************
  * @file           adc.h
  * @brief          header for adc.h
  * @author         D.Kerstens
  ******************************************************************************
  */
#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#define ADC_VREF  3300
#define ADC_FS    4096
#define R_TOP     10.0 // Top resistor in kohm of the voltage divider
#define R_BOTTOM  10.0 // Bottom resistor in kohm of the voltage divider
#define CONV_FACTOR_ADC (float) ((R_TOP + R_BOTTOM) * 3300) / (4096 * R_BOTTOM);

uint16_t ADC_Vsensor_Measure(ADC_HandleTypeDef *hadc);
uint16_t ADC_Vsensorslot_Measure(ADC_HandleTypeDef *hadc);

#endif /* ADC_ADC_H_ */
