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

typedef enum
{
  CRC_BigEndian,
  CRC_LittleEndian
}CRC_Endianness;

void ModbusInit(UART_HandleTypeDef *modbusHandle);
void ModbusEnableTX(void);
void ModbusDisableTX(void);
void ModbusSetBaudrate(uint32_t baudrate);

void ModbusTransmit(uint8_t *data, uint16_t size, CRC_Endianness endian);
void ModbusReceive(uint8_t *data, uint16_t size, CRC_Endianness endian);

void ModbusEchoTest(uint8_t slaveAddress, uint16_t data, uint8_t *response);

void ModbusWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t data);
void ModbusWriteMultipleRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data);

void ModbusReadInputRegisters(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data);
void ModbusReadHoldingRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t lenght, uint8_t *data);

#endif /* KELLER_MODBUS_H_ */
