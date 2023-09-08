/*
 * modbus.h
 *
 *  Created on: Sep 7, 2023
 *      Author: danny.kerstens
 */

#ifndef KELLER_MODBUS_H_
#define KELLER_MODBUS_H_

#include <stdint.h>
#include "main.h"

/* Modbus Function codes */
typedef enum
{
  FunctionReadCoils =             0x01,
  FunctionReadDiscreteInputs =    0x02,
  FunctionReadHoldingRegisters =  0x03,
  FunctionReadInputRegisters =    0x04,
  FunctionWriteSingleCoil =       0x05,
  FunctionWriteSingleRegister =   0x06,
  FunctionWriteMultipleCoils =    0x0F,
  FunctionWriteMultipleRegister = 0x10,
}ModBus_Function_t;

/* Modbus Exception codes */
typedef enum
{
  ExceptionIllegalFunction =         0x01,
  ExceptionIllegalDataAddress =      0x02,
  ExceptionIllegalDataValue =        0x03,
  ExceptionSlaveDeviceFailure =      0x04,
  ExceptionAcknowledge =             0x05,
  ExceptionSlaveDeviceBusy =         0x06,
  ExceptionMemoryParityError =       0x08,
  ExceptionGatewayPathUnavailable =  0x0A,
  ExceptionGatewayTargetNoResponse = 0x0B,
}ModBus_Errors_t;

void ModbusInit(UART_HandleTypeDef *modbusHandle);
void ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data);
void ModbusWriteMultipleRegister(uint16_t address, uint16_t lenght, uint16_t *data);

void ModbusReadInputRegisters(uint16_t address, uint8_t data);
void ModbusReadHoldingRegisters(uint16_t address, uint8_t lenght, uint8_t *data);

#endif /* KELLER_MODBUS_H_ */
