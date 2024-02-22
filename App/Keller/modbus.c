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

/**
 * @brief Initialize the Modbus UART handle
 */
void ModbusInit(UART_HandleTypeDef *modbusHandle)
{
  ModbusHandle = modbusHandle;
  ModbusShutdown(); //make sure transmitter and receiver are off.
}

/**
 * @brief Enable the external RS485 TX Transceiver ( and disable RX receiver )
 */
void ModbusEnableTX(void)
{
  HAL_GPIO_WritePin(RX_ENABLE_PORT, RX_ENABLE_PIN, GPIO_PIN_SET); //disable, low active
  HAL_GPIO_WritePin(TX_ENABLE_PORT, TX_ENABLE_PIN, GPIO_PIN_SET); //enable, high active
}

/**
 * @brief Disable the external RS485 TX Transceiver ( and enable RX receiver )
 */
void ModbusDisableTX(void)
{
  HAL_GPIO_WritePin(RX_ENABLE_PORT, RX_ENABLE_PIN, GPIO_PIN_RESET); //enable, low active
  HAL_GPIO_WritePin(TX_ENABLE_PORT, TX_ENABLE_PIN, GPIO_PIN_RESET); //disable, high active
}

/**
 * @brief Put the RS485 Driver in shutdown.
 * disabled RX, receiver
 * disabled TX, transmitter
 */
void ModbusShutdown(void)
{
  HAL_GPIO_WritePin(RX_ENABLE_PORT, RX_ENABLE_PIN, GPIO_PIN_SET);   //disable, low active
  HAL_GPIO_WritePin(TX_ENABLE_PORT, TX_ENABLE_PIN, GPIO_PIN_RESET); //disable, high active
}

/**
 * @brief Change the baudrate of the Modbus communication
 *
 * @param baudrate is the new baudrate for the Modbus communication
 */
void ModbusSetBaudrate(uint32_t baudrate)
{
  ModbusHandle->Init.BaudRate = baudrate;
  UART_SetConfig(ModbusHandle);
}

/**
 * @brief Flush the receive buffer
 */
void ModbusFlushRxBuffer(void)
{
  __HAL_UART_SEND_REQ(ModbusHandle, UART_RXDATA_FLUSH_REQUEST);
}

/**
 * @fn void ModbusTransmitData(uint8_t*, uint16_t)
 * @brief function to send an data array to the modbus uart directly
 *
 * @param data : pointer to data
 * @param length : length of data
 */
void ModbusTransmitData(uint8_t *data, uint16_t length)
{
  /* Flush the RX buffer */
  ModbusFlushRxBuffer();

  /* Transmit the message */
  ModbusEnableTX();
  HAL_UART_Transmit(ModbusHandle, data, length, MODBUS_TIMEOUT);
  ModbusDisableTX();
}

/**
 * @brief Transmit the Modbus message over the bus
 * function calculates the CRC16 and adds it to the transmitted data
 *
 * @param data is a pointer to the data buffer
 * @param size is the size of the message without the CRC size
 * @param endian is the endianness of the CRC
 */
void ModbusTransmit(uint8_t *data, uint16_t size, CRC_Endianness endian)
{
  /* Copy data into the message */
  uint8_t message[size + CRC_SIZE];
  for (uint8_t i = 0; i < size; i++)
    message[i] = data[i];

  /* Add the CRC to the message */
  static uint16_t crc;
  crc = calculateCRC_CCITT(data, size);
  if(endian == CRC_BIG_ENDIAN)
  {
    message[size] = (crc & 0x00FF);
    message[size + 1] = (crc & 0xFF00) >> 8;
  }
  else if (endian == CRC_LITTLE_ENDIAN)
  {
    message[size] = (crc & 0xFF00) >> 8;
    message[size + 1] = (crc & 0x00FF);
  }

  ModbusTransmitData(message, size + CRC_SIZE);
}

/**
 * @brief Receives the Modbus data on the bus
 *
 * @param data is a pointer to the receive buffer
 * @param size is the size of the message to receive
 * @param endian is the endianness of the CRC
 */
void ModbusReceive(uint8_t *data, uint16_t size, CRC_Endianness endian)
{
  /* Receive the modbus response */
  ModbusDisableTX();
  HAL_StatusTypeDef status = HAL_UART_Receive(ModbusHandle, data, size, MODBUS_TIMEOUT);
  if(status != HAL_OK)
  {
    memset(data, 0, size);
    return;
  }

  /* Check the CRC */
  static uint16_t crc;
  crc = calculateCRC_CCITT(data, size-CRC_SIZE);
  if(endian == CRC_BIG_ENDIAN)
  {
    if(crc != (data[size-1] << 8) + data[size-2])
    {
      memset(data, 0, size);
      return;
    }
  }
  else if (endian == CRC_LITTLE_ENDIAN)
  {
    if(crc != (data[size-2] << 8) + data[size-1])
    {
      memset(data, 0, size);
      return;
    }
  }
}

/**
 * @brief Write data to a single register
 *
 * @param slaveAddress is the address of the slave to write to
 * @param registerAddress is the address of the register to write to
 * @param data is the data to write to the register
 *
 * @return Modbus status
 */
MODBUS_StatusTypeDef ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data)
{
  uint8_t request[6];
  uint8_t respone[8];

  request[0] = slaveAddress;
  request[1] = FunctionWriteSingleRegister;
  request[2] = (registerAddress & 0xFF00) >> 8;
  request[3] = (registerAddress & 0x00FF);
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);

  ModbusTransmit(request, 6, CRC_LITTLE_ENDIAN);

  ModbusReceive(respone, 8, CRC_LITTLE_ENDIAN);

  /* Check for exceptions */
  if(respone[1] & 0x80)
    return respone[2];

  else
    return MODBUS_OK;
}

/**
 * @brief Write data to a multiple register
 *
 * @param slaveAddress is the address of the slave to write to
 * @param startAddress is the address of the first register to write to
 * @param lenght is the amount of register to write to
 * @param data is a pointer to the data to write to the registers
 *
 * @return Modbus status
 */
MODBUS_StatusTypeDef ModbusWriteMultipleRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data)
{
  uint8_t request[7 + lenght*2];
  uint8_t respone[8];

  request[0] = slaveAddress;
  request[1] = FunctionWriteMultipleRegister;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 123)
    return MODBUS_ILLEGAL_DATA_VALUE;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);
  request[6] = (lenght*2);

  for(uint8_t i=0; i<lenght; i++)
  {
    request[7+i*2] = (*(data+i) & 0xFF00) >> 8;
    request[8+i*2] = (*(data+i) & 0x00FF);
  }

  ModbusTransmit(request, 7 + lenght*2, CRC_LITTLE_ENDIAN);

  ModbusReceive(respone, 8, CRC_LITTLE_ENDIAN);

  /* Check for exceptions */
  if(respone[1] & 0x80)
    return respone[2];

  else
    return MODBUS_OK;
}

/**
 * @brief Read multiple holding registers
 *
 * @param slaveAddress is the address of the slave to read from
 * @param startAddress is the address of the first register to read from
 * @param lenght is the amount of register to read
 * @param data is a pointer to the receive buffer
 *
 * @return Modbus status
 */
MODBUS_StatusTypeDef ModbusReadHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data)
{
  uint8_t request[6];
  uint8_t response[3+lenght*2+2];

  request[0] = slaveAddress;
  request[1] = FunctionReadHoldingRegisters;
  request[2] = (startAddress & 0xFF00) >> 8;
  request[3] = (startAddress & 0x00FF);

  if(lenght > 125)
    return MODBUS_ILLEGAL_DATA_VALUE;

  request[4] = (lenght & 0xFF00) >> 8;
  request[5] = (lenght & 0x00FF);

  ModbusTransmit(request, 6, CRC_LITTLE_ENDIAN);
  ModbusReceive(response, 3+lenght*2+2, CRC_LITTLE_ENDIAN);

  /* Check for exceptions */
  if(response[1] & 0x80)
    return response[2];

  else
  {
    memcpy(data, response+3, 8);
  }
  return MODBUS_OK;
}

/**
 * @brief Echo the Modbus message
 *
 * @param slaveAddress is the address of the slave
 * @param data is the data to echo
 *
 * @return Modbus status
 */
MODBUS_StatusTypeDef ModbusEcho(uint8_t slaveAddress, uint16_t data)
{
  uint8_t request[6];
  uint8_t response[8];

  request[0] = slaveAddress;
  request[1] = FunctionDiagnostics;
  request[2] = 0x00;
  request[3] = 0x00;
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);


  ModbusTransmit(request, 6, CRC_LITTLE_ENDIAN);

  ModbusReceive(response, 8, CRC_LITTLE_ENDIAN);

  /* Check for exceptions */
  if(response[1] & 0x80)
    return response[2];

  else if(data != ((response[4] << 8) + response[5]))
    return MODBUS_ERROR;

  else
    return MODBUS_OK;
}

#endif /* KELLER_MODBUS_C_ */
