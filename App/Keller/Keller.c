
#include <string.h>
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"


static uint32_t count_illegalNonImplementedFunction;
static uint32_t count_illegalDataAddress;
static uint32_t count_illegalDataValue;
static uint32_t count_slaveDeviceFailure;
static uint32_t count_noInitialisation;

/**
 * @fn void KellerErrorResponse(uint8_t)
 * @brief function to deal with an error exception for debug
 *
 * @param data
 */
void KellerErrorResponse(uint8_t data)
{
  switch (data )
  {
    case 0x01: //illegal non-implemented function 1
      assert_param(0);
      count_illegalNonImplementedFunction++;
      break;
    case 0x02: //illegal data address 2
      assert_param(0);
      count_illegalDataAddress++;
      break;
    case 0x03: //illegal data value 3
      assert_param(0);
      count_illegalDataValue++;
      break;
    case 0x04: //slave device failure 4
      assert_param(0);
      count_slaveDeviceFailure++;
      break;
    case 0x20: //initialisation (only KELLER bus) 32
      assert_param(0);
      count_noInitialisation++;
      break;
    default:
      break;
  }
}

/**
 * @fn bool KellerVerifyResultOkay(uint8_t*, uint8_t)
 * @brief function to verify keller response
 *
 * @param response
 * @param functionCode
 * @return
 */
bool KellerVerifyResultOkay(uint8_t * response, uint8_t functionCode)
{
  if( response[1] == functionCode)
  {
    return true;
  }

  //else failed The slave responds with an exception error

  if( response[1] & 0x80 )
  {
    KellerErrorResponse(response[2]);
  }
  return false;
}


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
 * @fn bool KellerWriteConfig(uint8_t, uint8_t, uint8_t)
 * @brief Write the configuration register
 *
 * @param slaveAddress The slave address of the sensor
 * @param configNumber The number of the configuration register
 * @param data The data to write to the register
 * @return true = succeed, false = failed
 */
bool KellerWriteConfig(uint8_t slaveAddress, uint8_t configNumber, uint8_t data)
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


  //verify result
  if( KellerVerifyResultOkay(response, request[1]) )
  {
    return response[2] == 0; //return valied if response value is 0
  }

  return 0; //error return 0
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
 * @fn bool KellerSetBaudrate(uint8_t, Keller_Baudrate_t)
 * @brief Change the baudrate of the Keller sensor
 *
 * @param slaveAddress The slave address of the sensor
 * @param baudrate The baudrate to set the Keller sensor to
 * @return true = succeed, false = failed
 */
bool KellerSetBaudrate(uint8_t slaveAddress, Keller_Baudrate_t baudrate)
{
  bool result = 0;
  if(baudrate == BAUD_115200)
  {
    ModbusSetBaudrate(9600);
    result = KellerWriteConfig(slaveAddress, CONFIG_UART, BAUD_115200);
    ModbusSetBaudrate(115200);
  }
  else if(baudrate == BAUD_9600)
  {
    ModbusSetBaudrate(115200);
    result = KellerWriteConfig(slaveAddress, CONFIG_UART, BAUD_9600);
    ModbusSetBaudrate(9600);
  }

  return result;
}


/**
 * @fn bool KellerCheckBaudrate(uint8_t, Keller_Baudrate_t)
 * @brief function to verify the current baudrate.
 *
 * @param slaveAddress
 * @param baudrate
 * @return
 */
bool KellerCheckBaudrate(uint8_t slaveAddress, Keller_Baudrate_t baudrate)
{
  uint32_t currentBaudrate = ModbusGetBaudrate();
  if(baudrate == BAUD_115200)
  {
    ModbusSetBaudrate(115200);
  }
  else if (baudrate == BAUD_9600)
  {
    ModbusSetBaudrate(9600);
  }

  uint8_t data[1] = { 0xFF };
  ModbusTransmitData(data, sizeof(data));
  HAL_Delay(2);
  bool status = KellerInit(250);

  ModbusSetBaudrate(currentBaudrate); //switch back to previous baudrate.

  return status;
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

  //verify result
  if( KellerVerifyResultOkay(response, request[1]) )
  {
    return response[2]; //return new address
  }

  return 0; //error return 0
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
