/**
  ******************************************************************************
  * @file           I2C_Slave.h
  * @brief          header for I2C_Slave.c
  * @author         D.Kerstens
  ******************************************************************************
  */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#include "main.h"

extern volatile bool writeFlag;
extern uint8_t regWriteData[5];
extern uint8_t regSize;
extern volatile bool sensorSlaveErrorFlag;

void sensorSlaveTransmit(uint8_t *data, uint8_t size);

#endif /* I2C_SLAVE_H_ */
