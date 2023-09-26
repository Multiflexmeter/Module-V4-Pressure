
#include "Keller.h"
#include "modbus.h"
#include "crc16.h"

void KellerEchoTest(uint8_t slaveAddress, uint16_t data, uint8_t *response)
{
  uint8_t request[8];

  request[0] = slaveAddress;
  request[1] = FunctionDiagnostics;
  request[2] = 0x00;
  request[3] = 0x00;
  request[4] = (data & 0xFF00) >> 8;
  request[5] = (data & 0x00FF);

  static uint16_t crc;
  crc = calculateCRC_CCITT(request, 6);
  request[6] = (crc & 0xFF00) >> 8;
  request[7] = (crc & 0x00FF);

  ModbusTransmit(request, 8);

  ModbusReceive(response, 8);
}
//void Keller_WriteConfiguration(uint8_t deviceAddress, uint16_t startAddress, uint16_t lenght, uint16_t *data)
