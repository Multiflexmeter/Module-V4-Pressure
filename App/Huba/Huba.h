/**
  ******************************************************************************
  * @file           Huba.h
  * @brief          header for Huba.c
  * @author         D.Kerstens
  ******************************************************************************
  */

#ifndef HUBA_HUBA_H_
#define HUBA_HUBA_H_

#include <stdbool.h>
#include "SensorRegister.h"

#define TIMEBUFFER_SIZE 30
#define CLOCK_FREQUENCY

/**
 * @struct HubaSensor
 * @brief setting and data structure for Huba measurement
 *
 */
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

void hubaStart(HubaSensor *sensor);
SensorDataHuba hubaBufferToData(HubaSensor *sensor);
void hubaTimerCallback(HubaSensor *sensor);

#endif /* HUBA_HUBA_H_ */
