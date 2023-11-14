
#ifndef HUBA_HUBA_H_
#define HUBA_HUBA_H_

#include "SensorRegister.h"
#include <stdbool.h>

#define TIMEBUFFER_SIZE 30
#define CLOCK_FREQUENCY

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t strobeTimeStart;
  uint32_t strobeTimeEnd;
  uint8_t bitIndex;
  uint8_t hubaBuf[30];
  uint8_t tmpBuf[30];
  bool firstCapture;
  bool hubaDone;
}HubaSensor;

void hubaInit(HubaSensor *sensor);
SensorData hubaBufferToData(HubaSensor *sensor);
void hubaTimerCallback(HubaSensor *sensor);

#endif /* HUBA_HUBA_H_ */
