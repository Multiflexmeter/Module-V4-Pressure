
#include <string.h>
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"


/* Keller Private Functions */

float KellerArrayToFloat(uint8_t *array)
{
  float f;
  uint32_t b = ((uint32_t) array[0] << 24) | ((uint32_t) array[1] << 16) | ((uint32_t) array[2] << 8) | ((uint32_t) array[3]);
  memcpy(&f , &b, sizeof(float));
  return f;
}
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
bool KellerInit(uint8_t slaveAddress)
{
  uint8_t request[2];
  uint8_t response[10];

  request[0] = slaveAddress;
  request[1] = FunctionInitialiseDevices;

  // Transmit command and receive response
  ModbusTransmit(request, 2, CRC_BIG_ENDIAN);
  ModbusReceive(response, 10, CRC_BIG_ENDIAN);

  if(response[1] == FunctionInitialiseDevices)
    return true;
  else
    return false;
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
  if(baudrate == BAUD_115200)
  {
    ModbusSetBaudrate(9600);
    KellerWriteConfig(slaveAddress, CONFIG_UART, BAUD_115200);
    ModbusSetBaudrate(115200);
  }
  else if(baudrate == BAUD_9600)
  {
    ModbusSetBaudrate(115200);
    KellerWriteConfig(slaveAddress, CONFIG_UART, BAUD_9600);
    ModbusSetBaudrate(9600);
  }

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


/**
 * @brief Read the pressure from the Keller sensor
 *
 * @param slaveAddress The slave address of the sensor
 *
 * @return Pressure in Pascal
 */
int32_t KellerReadPressure(uint8_t slaveAddress)
{
  return KellerReadChannelInt(slaveAddress, CHANNEL_P1);
}

/**
 * @brief Read the temperature from the Keller sensor
 *
 * @param slaveAddress The slave address of the sensor
 *
 * @return Temperature with resolution of 0.01°C
 */
int32_t KellerReadTemperature(uint8_t slaveAddress)
{
  return KellerReadChannelInt(slaveAddress, CHANNEL_TOB1);
}

SensorData KellerReadTempAndPressure(uint8_t slaveAddress)
{
  SensorData sensorData;
  uint8_t rxBuffer[8];

  ModbusReadHoldingRegister(slaveAddress, 0x100, 0x04, rxBuffer);

  sensorData.pressure = KellerArrayToFloat(rxBuffer);
  sensorData.temperature = KellerArrayToFloat(rxBuffer+4);

  return sensorData;
}
