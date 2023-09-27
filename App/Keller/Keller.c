
#include <string.h>
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"


void KellerInit(uint8_t slaveAddress)
{
  uint8_t request[2];
  uint8_t response[10];

  request[0] = slaveAddress;
  request[1] = FunctionInitialiseDevices;

  ModbusTransmit(request, 2, CRC_BigEndian);

  ModbusReceive(response, 10, CRC_BigEndian);
}

uint32_t KellerSerialnumber(uint8_t slaveAddress)
{
  uint8_t request[2];
  uint8_t response[8];

  request[0] = slaveAddress;
  request[1] = FunctionReadSerialNumber;

  // Transmit command and receive response
  ModbusTransmit(request, 2, CRC_BigEndian);

  ModbusReceive(response, 8, CRC_BigEndian);

  return (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
}

uint8_t KellerReadConfig(uint8_t slaveAddress, uint8_t configNumber)
{
  uint8_t request[3];
  uint8_t response[8];

  request[0] = slaveAddress;
  request[1] = FunctionReadConfigurations;
  request[2] = configNumber;

  // Transmit command and receive response
  ModbusTransmit(request, 3, CRC_BigEndian);

  ModbusReceive(response, 5, CRC_BigEndian);

  return response[2];
}

void KellerWriteConfig(uint8_t slaveAddress, uint8_t configNumber, uint8_t data)
{
  uint8_t request[4];
  uint8_t response[8];

  request[0] = slaveAddress;
  request[1] = FunctionWriteConfigurations;
  request[2] = configNumber;
  request[3] = data;

  // Transmit command and receive response
  ModbusTransmit(request, 4, CRC_BigEndian);

  ModbusReceive(response, 5, CRC_BigEndian);

  return;
}

void KellerSetBaudrate(uint8_t slaveAddress, Keller_Baudrate_t baudrate)
{
  uint8_t uartConfig;

  uartConfig = KellerReadConfig(slaveAddress, 10);

  KellerWriteConfig(slaveAddress, 10, (uartConfig & 0xF0) | baudrate);

  if(baudrate == BAUD_115200)
    ModbusSetBaudrate(115200);
  else if(baudrate == BAUD_9600)
    ModbusSetBaudrate(9600);

  KellerInit(slaveAddress);
  return;
}

uint8_t KellerNewAddress(uint8_t currentSlaveAddress, uint8_t newSlaveAddress)
{
  uint8_t request[3];
  uint8_t response[5];

  request[0] = currentSlaveAddress;
  request[1] = FunctionWriteDeviceAddress;
  request[2] = newSlaveAddress;

  ModbusTransmit(request, 3, CRC_BigEndian);

  ModbusReceive(response, 5, CRC_BigEndian);

  return response[2];
}

float KellerReadChannelFloat(uint8_t slaveAddress, uint8_t channel)
{
  uint8_t request[3];
  uint8_t response[9];

  request[0] = slaveAddress;
  request[1] = FunctionReadChannelFloat;
  request[2] = channel;

  ModbusTransmit(request, 3, CRC_BigEndian);

  ModbusReceive(response, 9, CRC_BigEndian);

  uint32_t vBuffer = ((uint32_t) response[2] << 24) | ((uint32_t) response[3] << 16) | ((uint32_t) response[4] << 8) | ((uint32_t) response[5]);
  float result;
  memcpy(&result, &vBuffer, sizeof(result));

  return result;
}

uint32_t KellerReadChannelInt(uint8_t slaveAddress, uint8_t channel)
{
  uint8_t request[3];
  uint8_t response[9];

  request[0] = slaveAddress;
  request[1] = FunctionReadShannelInt;
  request[2] = channel;

  ModbusTransmit(request, 3, CRC_BigEndian);

  ModbusReceive(response, 9, CRC_BigEndian);

  return (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
}

void KellerEchoTest(uint8_t slaveAddress, uint16_t data, uint8_t *response)
{
  uint8_t request[6];

  request[0] = slaveAddress;
  request[1] = FunctionDiagnostics;
  request[2] = 0x00;
  request[3] = 0x00;
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);


  ModbusTransmit(request, 6, CRC_LittleEndian);

  ModbusReceive(response, 8, CRC_LittleEndian);
}
//void Keller_WriteConfiguration(uint8_t deviceAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data)
