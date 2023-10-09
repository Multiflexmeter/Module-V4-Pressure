
#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#define ADC_VREF  3300
#define ADC_FS    4096
#define R_TOP     10000.0 // Top resistor of the voltage divider
#define R_BOTTOM  10000.0 // Bottom resistor of the voltage divider
#define CONV_FACTOR_ADC (float) ((R_TOP + R_BOTTOM) * 3300) / (4096 * R_BOTTOM);


void ADC_Start(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* ADC_ADC_H_ */
