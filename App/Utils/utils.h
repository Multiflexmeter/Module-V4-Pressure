/**
  ******************************************************************************
  * @file           Utils.h
  * @brief          header for Utils.c
  * @author         D.Kerstens
  * @date           Nov 9, 2023
  ******************************************************************************
  */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

#include "main.h"
#include <stdbool.h>

typedef enum
{
  RS485_VARIANT = 0,
  ONEWIRE_VARIANT = 1,
} variant_t;

variant_t getVariant(void);
void setSlaveAddress(void);
float findMedian_float(float a[], uint8_t n);
uint8_t findMedian_uint8(uint8_t a[], uint8_t n);
uint16_t findMedian_uint16(uint16_t a[], uint8_t n);
void enter_Sleep(void);
void determineSensorType(void);

void controlSensor1(GPIO_PinState state);
void controlSensor2(GPIO_PinState state);
void enableSensor1(void);
void enableSensor2(void);
void disableSensors(void);

void controlBuckConverter(GPIO_PinState state);
void switchOnSensor_BothKeller(void);

bool assignAddressKellerBothSensors(void);
bool assignAddressKeller(uint8_t sensor);
void measureKellerSensor(void);
void measureHubaSensor(void);

#endif /* UTILS_UTILS_H_ */
