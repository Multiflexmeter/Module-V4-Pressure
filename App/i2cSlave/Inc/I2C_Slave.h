/*
 * I2C_Slave.h
 *
 *  Created on: Jul 11, 2023
 *      Author: danny.kerstens
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#include "main.h"

extern bool writeFlag;
extern uint8_t regWriteData[5];
extern uint8_t regSize;
extern volatile bool sensorSlaveErrorFlag;

void sensorSlaveTransmit(uint8_t *data, uint8_t size);

#endif /* I2C_SLAVE_H_ */
