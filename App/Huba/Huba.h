
#ifndef HUBA_HUBA_H_
#define HUBA_HUBA_H_

#include "SensorRegister.h"
#include <stdbool.h>

#define TIMEBUFFER_SIZE 30
#define CLOCK_FREQUENCY


void hubaInit(TIM_HandleTypeDef *htim);
SensorData hubaBufferToData(void);
void hubaTimerCallback(TIM_HandleTypeDef *htim);

#endif /* HUBA_HUBA_H_ */
