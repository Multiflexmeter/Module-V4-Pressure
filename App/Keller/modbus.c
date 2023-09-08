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

void ModbusWriteMultipleRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data)
{
  uint8_t request[7 + lenght*2 + CRC_SIZE];

  request[0] = slaveAddress;
  request[1] = FunctionWriteMultipleRegister;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 123)
    return;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);
  request[6] = (lenght*2);

  for(uint8_t i=0; i<lenght; i++)
  {
    request[7+i*2] = (*(data+i) & 0xFF00) >> 8;
    request[8+i*2] = (*(data+i) & 0x00FF);
  }

  static uint16_t crc;
  crc = calculateCRC_CCITT(request, 7 + lenght*2);
  request[7 + lenght*2] = (crc & 0xFF00) >> 8;
  request[8 + lenght*2] = (crc & 0x00FF);

  HAL_UART_Transmit(ModbusHandle, request, 7 + lenght*2 + CRC_SIZE, 100);
}

void ModbusReadInputRegisters(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data)
{
  uint8_t request[8];

  request[0] = slaveAddress;
  request[1] = FunctionReadInputRegisters;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 125)
    return;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);

  static uint16_t crc;
  crc = calculateCRC_CCITT(request, 6);
  request[6] = (crc & 0xFF00) >> 8;
  request[7] = (crc & 0x00FF);

  HAL_UART_Transmit(ModbusHandle, request, 8, 100);
  HAL_UART_Receive(ModbusHandle, data, 3+lenght*2, 100);
}


void ModbusReadHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data)
{
  uint8_t request[8];

  request[0] = slaveAddress;
  request[1] = FunctionReadHoldingRegisters;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 125)
    return;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);

  static uint16_t crc;
  crc = calculateCRC_CCITT(request, 6);
  request[6] = (crc & 0xFF00) >> 8;
  request[7] = (crc & 0x00FF);

  HAL_UART_Transmit(ModbusHandle, request, 8, 100);
  HAL_UART_Receive(ModbusHandle, data, 3+lenght*2, 100);
}

#endif /* KELLER_MODBUS_C_ */
