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

#define CRC_SIZE                  2
#define REQUEST_SIZE              6

/* User defined enable pins */
#define RX_ENABLE_PORT            USART_RX_Enable_GPIO_Port
#define RX_ENABLE_PIN             USART_RX_Enable_Pin

#define TX_ENABLE_PORT            USART_TX_Enable_GPIO_Port
#define TX_ENABLE_PIN             USART_TX_Enable_Pin

#define MODBUS_TIMEOUT            100

/* Modbus Function codes */
typedef enum
{
  FunctionReadCoils =             0x01,
  FunctionReadDiscreteInputs =    0x02,
  FunctionReadHoldingRegisters =  0x03,
  FunctionReadInputRegisters =    0x04,
  FunctionWriteSingleCoil =       0x05,
  FunctionWriteSingleRegister =   0x06,
  FunctionDiagnostics =           0x08,
  FunctionWriteMultipleCoils =    0x0F,
  FunctionWriteMultipleRegister = 0x10,
}MODBUS_FunctionTypeDef;

/* Modbus Exception codes */
typedef enum
{
  MODBUS_ERROR =                  -1,
  MODBUS_OK =                     0,
  MODBUS_ILLEGAL_FUNCTION =       1,
  MODBUS_ILLEGAL_DATA_ADDRESS =   2,
  MODBUS_ILLEGAL_DATA_VALUE =     3,
  MODBUS_SLAVE_DEVICE_FAILURE =   4,
}MODBUS_StatusTypeDef;

typedef enum
{
  CRC_BIG_ENDIAN,
  CRC_LITTLE_ENDIAN
}CRC_Endianness;


void ModbusInit(UART_HandleTypeDef *modbusHandle);
void ModbusEnableTX(void);
void ModbusDisableTX(void);
void ModbusSetBaudrate(uint32_t baudrate);

void ModbusTransmit(uint8_t *data, uint16_t size, CRC_Endianness endian);
void ModbusReceive(uint8_t *data, uint16_t size, CRC_Endianness endian);

MODBUS_StatusTypeDef ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data);
MODBUS_StatusTypeDef ModbusWriteMultipleRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data);
MODBUS_StatusTypeDef ModbusReadHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data);
MODBUS_StatusTypeDef ModbusEcho(uint8_t slaveAddress, uint16_t data);

#endif /* KELLER_MODBUS_H_ */
