
#include <string.h>
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"


/* Keller Private Functions */
/**
 * @brief Read the configuration register
 *
 * @param slaveAddress The slave address of the sensor
 * @param configNumber The number of the configuration register
 *
 * @return The register value
 */
uint8_t KellerReadConfig(uint8_t slaveAddress, uint8_t configNumber)
{
  uint8_t request[3];
  uint8_t response[8];

  // Fill the Modbus request command
  request[0] = slaveAddress;
  request[1] = FunctionReadConfigurations;
  request[2] = configNumber;

  // Transmit command and receive response
  ModbusTransmit(request, 3, CRC_BIG_ENDIAN);
  ModbusReceive(response, 5, CRC_BIG_ENDIAN);

  return response[2];
}

/**
 * @brief Write the configuration register
 *
 * @param slaveAddress The slave address of the sensor
 * @param configNumber The number of the configuration register
 * @param data The data to write to the register
 */
void KellerWriteConfig(uint8_t slaveAddress, uint8_t configNumber, uint8_t data)
{
  uint8_t request[4];
  uint8_t response[8];

  // Fill the Modbus request command
  request[0] = slaveAddress;
  request[1] = FunctionWriteConfigurations;
  request[2] = configNumber;
  request[3] = data;

  // Transmit command and receive response
  ModbusTransmit(request, 4, CRC_BIG_ENDIAN);
  ModbusReceive(response, 5, CRC_BIG_ENDIAN);

  return;
}

/**
 * @brief Read the value of a sensor channel in the float format
 *
 * @param slaveAddress The slave address of the sensor
 * @param channel The sensor channel to read
 *
 * @return The sensor channel data as an float
 */
float KellerReadChannelFloat(uint8_t slaveAddress, uint8_t channel)
{
  uint8_t request[3];
  uint8_t response[9];

  // Fill the Modbus request command
  request[0] = slaveAddress;
  request[1] = FunctionReadChannelFloat;
  request[2] = channel;

  // Transmit command and receive response
  ModbusTransmit(request, 3, CRC_BIG_ENDIAN);
  ModbusReceive(response, 9, CRC_BIG_ENDIAN);

  uint32_t vBuffer = ((uint32_t) response[2] << 24) | ((uint32_t) response[3] << 16) | ((uint32_t) response[4] << 8) | ((uint32_t) response[5]);
  float result;
  memcpy(&result, &vBuffer, sizeof(result));

  return result;
}

/**
 * @brief Read the value of a sensor channel in the in32_t format
 *
 * @param slaveAddress The slave address of the sensor
 * @param channel The sensor channel to read
 *
 * @return The sensor channel data as an int32_t
 */
int32_t KellerReadChannelInt(uint8_t slaveAddress, uint8_t channel)
{
  uint8_t request[3];
  uint8_t response[9];

  // Fill the Modbus request command
  request[0] = slaveAddress;
  request[1] = FunctionReadShannelInt;
  request[2] = channel;

  // Transmit command and receive response
  ModbusTransmit(request, 3, CRC_BIG_ENDIAN);
  ModbusReceive(response, 9, CRC_BIG_ENDIAN);

  return (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
}


/* Keller Functions */
/**
 * @brief Initialize the Keller sensor
 *
 * @param slaveAddress The slave address of the sensor
 */
void KellerInit(uint8_t slaveAddress)
{
  uint8_t request[2];
  uint8_t response[10];

  request[0] = slaveAddress;
  request[1] = FunctionInitialiseDevices;

  // Transmit command and receive response
  ModbusTransmit(request, 2, CRC_BIG_ENDIAN);
  ModbusReceive(response, 10, CRC_BIG_ENDIAN);
}

/**
 * @brief Reads out the Keller sensor serial number
 *
 * @param slaveAddress The slave address of the sensor
 *
 * @return The sensor serial number
 */
uint32_t KellerSerialnumber(uint8_t slaveAddress)
{
  uint8_t request[2];
  uint8_t response[8];

  // Fill the Modbus request command
  request[0] = slaveAddress;
  request[1] = FunctionReadSerialNumber;

  // Transmit command and receive response
  ModbusTransmit(request, 2, CRC_BIG_ENDIAN);
  ModbusReceive(response, 8, CRC_BIG_ENDIAN);

  return (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
}

/**
 * @brief Change the baudrate of the Keller sensor
 *
 * @param slaveAddress The slave address of the sensor
 * @param baudrate The baudrate to set the Keller sensor to
 */
void KellerSetBaudrate(uint8_t slaveAddress, Keller_Baudrate_t baudrate)
{
  uint8_t uartConfig;

  ModbusSetBaudrate(9600);
  uartConfig = KellerReadConfig(slaveAddress, 10);
  KellerWriteConfig(slaveAddress, 10, (uartConfig & 0xF0) | baudrate);

  ModbusSetBaudrate(115200);
  uartConfig = KellerReadConfig(slaveAddress, 10);
  KellerWriteConfig(slaveAddress, 10, (uartConfig & 0xF0) | baudrate);

  if(baudrate == BAUD_115200)
    ModbusSetBaudrate(115200);
  else if(baudrate == BAUD_9600)
    ModbusSetBaudrate(9600);

  KellerInit(slaveAddress);
  return;
}

/**
 * @brief Change the slave address off the Keller sensor
 *
 * @param currentSlaveAddress The current slave address of the Keller sensor
 * @param newSlaveAddress The new slave address for the Keller sensor
 *
 * @return The new slave address
 */
uint8_t KellerNewAddress(uint8_t currentSlaveAddress, uint8_t newSlaveAddress)
{
  uint8_t request[3];
  uint8_t response[5];

  // Fill the Modbus request command
  request[0] = currentSlaveAddress;
  request[1] = FunctionWriteDeviceAddress;
  request[2] = newSlaveAddress;

  // Transmit command and receive response
  ModbusTransmit(request, 3, CRC_BIG_ENDIAN);
  ModbusReceive(response, 5, CRC_BIG_ENDIAN);

  return response[2];
}

