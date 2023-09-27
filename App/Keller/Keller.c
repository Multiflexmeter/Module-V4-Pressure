
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"

void KellerInit(uint8_t slaveAddress, uint8_t *response)
{
  uint8_t request[2];

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

  ModbusTransmit(request, 2, CRC_BigEndian);

  ModbusReceive(response, 8, CRC_BigEndian);

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
