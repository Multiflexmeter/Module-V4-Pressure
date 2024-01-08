/*
 * utils.h
 *
 *  Created on: Nov 9, 2023
 *      Author: danny.kerstens
 */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

#include "main.h"

typedef enum
{
  RS485_VARIANT = 0,
  ONEWIRE_VARIANT = 1,
} variant_t;

variant_t getVariant(void);
void setSlaveAddress(void);
float findMedian(float a[], uint8_t n);
void enter_Sleep(void);

void enableSensor1(void);
void enableSensor2(void);
void enableSensors(void);
void disableSensors(void);

void assignAddressKeller(void);
void measureKellerSensor(void);

#endif /* UTILS_UTILS_H_ */
