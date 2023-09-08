/*
 * modbus.c
 *
 *  Created on: Sep 7, 2023
 *      Author: danny.kerstens
 */

#ifndef KELLER_MODBUS_C_
#define KELLER_MODBUS_C_

#include "modbus.h"
#include "crc16.h"

static UART_HandleTypeDef *ModbusHandle;

void ModbusInit(UART_HandleTypeDef *modbusHandle)
{
  ModbusHandle = modbusHandle;
}

void ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data)
{
  uint8_t request[8];

  request[0] = slaveAddress;
  request[1] = FunctionWriteSingleRegister;
  request[2] = (registerAddress & 0xFF00) >> 8;
  request[3] = (registerAddress & 0x00FF);
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);

  static uint16_t crc;
  crc = calculateCRC_CCITT(request, 6);
  request[6] = (crc & 0xFF00) >> 8;
  request[7] = (crc & 0x00FF);

  HAL_UART_Transmit(ModbusHandle, request, 8, 100);
}

void ModbusWriteMultipleRegister(uint16_t address, uint16_t lenght, uint16_t *data)
{

}

#endif /* KELLER_MODBUS_C_ */
