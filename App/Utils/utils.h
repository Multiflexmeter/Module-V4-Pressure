/*
 * utils.h
 *
 *  Created on: Nov 9, 2023
 *      Author: danny.kerstens
 */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

#include "main.h"
#include "SensorRegister.h"

void setSlaveAddress(void);
float findMedian(float a[], uint8_t n);
void enter_Sleep(void);
void determineSensorType(void);

#endif /* UTILS_UTILS_H_ */
