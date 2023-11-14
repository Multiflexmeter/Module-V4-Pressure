
#ifndef HUBA_HUBA_H_
#define HUBA_HUBA_H_

#include "SensorRegister.h"

#define TIMEBUFFER_SIZE 30
#define CLOCK_FREQUENCY

SensorData hubaBufferToData(const uint8_t *buffer);

#endif /* HUBA_HUBA_H_ */
