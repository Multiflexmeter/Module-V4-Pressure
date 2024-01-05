/*
 * utils.h
 *
 *  Created on: Nov 9, 2023
 *      Author: danny.kerstens
 */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

#include "main.h"

void setSlaveAddress(void);
float findMedian(float a[], uint8_t n);
void enter_Sleep(void);

void enableSensor1(void);
void enableSensor2(void);
void enableSensors(void);
void disableSensors(void);

#endif /* UTILS_UTILS_H_ */
