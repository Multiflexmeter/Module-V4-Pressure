/*
 * modbus.c
 *
 *  Created on: Sep 7, 2023
 *      Author: danny.kerstens
 */

#ifndef KELLER_MODBUS_C_
#define KELLER_MODBUS_C_

#include <string.h>
#include "modbus.h"
#include "crc16.h"

static UART_HandleTypeDef *ModbusHandle;

void ModbusInit(UART_HandleTypeDef *modbusHandle)
{
  ModbusHandle = modbusHandle;
}

void ModbusEnableTX(void)
{
  HAL_GPIO_WritePin(RX_ENABLE_PORT, RX_ENABLE_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TX_ENABLE_PORT, TX_ENABLE_PIN, GPIO_PIN_SET);
}

void ModbusDisableTX(void)
{
  HAL_GPIO_WritePin(RX_ENABLE_PORT, RX_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TX_ENABLE_PORT, TX_ENABLE_PIN, GPIO_PIN_RESET);
}

void ModbusTransmit(uint8_t *data, uint16_t size, CRC_Endianness endian)
{
  // Copy data into the message
  uint8_t message[size + CRC_SIZE];
  for (uint8_t i = 0; i < size; i++)
    message[i] = data[i];

  // Add the CRC to the message
  static uint16_t crc;
  crc = calculateCRC_CCITT(data, size);
  if(endian == CRC_BigEndian)
  {
    message[size] = (crc & 0x00FF);
    message[size + 1] = (crc & 0xFF00) >> 8;
  }
  else if (endian == CRC_LittleEndian)
  {
    message[size] = (crc & 0xFF00) >> 8;
    message[size + 1] = (crc & 0x00FF);
  }


  // Transmit the message
  ModbusEnableTX();
  HAL_UART_Transmit(ModbusHandle, message, size + CRC_SIZE, MODBUS_TIMEOUT);
  ModbusDisableTX();
}

void ModbusReceive(uint8_t *data, uint16_t size, CRC_Endianness endian)
{
  // Receive the modbus response
  ModbusDisableTX();
  HAL_UART_Receive(ModbusHandle, data, size, MODBUS_TIMEOUT);

  // Check the CRC
  static uint16_t crc;
  crc = calculateCRC_CCITT(data, size-CRC_SIZE);
  if(endian == CRC_BigEndian)
  {
    if(crc != (data[size-1] << 8) + data[size-2])
    {
      memset(data, 0, size);
    }
  }
  else if (endian == CRC_LittleEndian)
  {
    if(crc != (data[size-2] << 8) + data[size-1])
    {
      memset(data, 0, size);
    }
  }
}

void ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data)
{
  uint8_t request[6];

  request[0] = slaveAddress;
  request[1] = FunctionWriteSingleRegister;
  request[2] = (registerAddress & 0xFF00) >> 8;
  request[3] = (registerAddress & 0x00FF);
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);

  ModbusTransmit(request, 6, CRC_LittleEndian);
}

void ModbusWriteMultipleRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data)
{
  uint8_t request[7 + lenght*2];

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

  ModbusTransmit(request, 7 + lenght*2, CRC_LittleEndian);
}

void ModbusReadInputRegisters(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data)
{
  uint8_t request[6];

  request[0] = slaveAddress;
  request[1] = FunctionReadInputRegisters;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 125)
    return;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);

  ModbusTransmit(request, 6, CRC_LittleEndian);
  ModbusReceive(data, 3+lenght*2, CRC_LittleEndian);
}


void ModbusReadHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data)
{
  uint8_t request[6];

  request[0] = slaveAddress;
  request[1] = FunctionReadHoldingRegisters;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 125)
    return;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);

  ModbusTransmit(request, 6, CRC_LittleEndian);
  ModbusReceive(data, 3+lenght*2, CRC_LittleEndian);
}

#endif /* KELLER_MODBUS_C_ */
