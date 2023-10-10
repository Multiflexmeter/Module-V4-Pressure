
#include <stdint.h>
#include <string.h>
#include "SensorRegister.h"
#include "crc16.h"

/* Register Declaration */
static uint8_t registerFirmwareVersion[10];
static uint8_t registerProtocolVersion = DEF_PROTOCOL_VERSION;
static uint16_t registerSensorType = DEF_SENSOR_TYPE;
static uint8_t registerMeasurementStart = DEF_MEAS_START;
static uint8_t registerMeasurementStatus = DEF_MEAS_STATUS;
static uint16_t registerMeasurementTime = DEF_MEAS_TIME;
static uint8_t registerMeasurementSize = DEF_MEAS_SIZE;
static int32_t registerMeasurementData[2] = {DEF_MEAS_DATA, DEF_MEAS_DATA};
static uint8_t registerSensorAmount = DEF_SENSOR_AMOUNT;
static uint8_t registerSensorSelected = DEF_SENSOR_SELECTED;
static uint8_t registerMeasurementType = DEF_MEAS_TYPE;
static uint16_t registerMeasurementSamples = DEF_MEAS_SAMPLES;
static uint8_t registerSensorUnit = DEF_SENSOR_UNIT;
static uint8_t registerSensorSize = DEF_SENSOR_SIZE;
static uint16_t registerSensorData = DEF_SENSOR_DATA;
static uint16_t registerErrorCounter[3] = {DEF_ERROR_COUNT, DEF_ERROR_COUNT, DEF_ERROR_COUNT};
static uint8_t registerErrorStatus = DEF_ERROR_STATUS;

const SensorReg registers[] =
{
    {REG_FIRMWARE_VERSION,  &registerFirmwareVersion,     UINT8_T,  10, READ},
    {REG_PROTOCOL_VERSION,  &registerProtocolVersion,     UINT8_T,  1,  READ},
    {REG_SENSOR_TYPE,       &registerSensorType,          UINT16_T, 1,  READ},
    {REG_MEAS_START,        &registerMeasurementStart,    UINT8_T,  1,  READWRITE},
    {REG_MEAS_STATUS,       &registerMeasurementStatus,   UINT8_T,  1,  READ},
    {REG_MEAS_TIME,         &registerMeasurementTime,     UINT16_T, 1,  READWRITE},
    {REG_MEAS_SIZE,         &registerMeasurementSize,     UINT8_T,  1,  READ},
    {REG_MEAS_DATA,         &registerMeasurementData,     INT32_T,  2,  READ},
    {REG_SENSOR_AMOUNT,     &registerSensorAmount,        UINT8_T,  1,  READ},
    {REG_SENSOR_SELECTED,   &registerSensorSelected,      UINT8_T,  1,  READWRITE},
    {REG_MEAS_TYPE,         &registerMeasurementType,     UINT8_T,  1,  READWRITE},
    {REG_MEAS_SAMPLES,      &registerMeasurementSamples,  UINT16_T, 1,  READWRITE},
    {REG_SENSOR_UNIT,       &registerSensorUnit,          UINT8_T,  1,  READ},
    {REG_SENSOR_SIZE,       &registerSensorSize,          UINT8_T,  1,  READ},
    {REG_SENSOR_DATA,       &registerSensorData,          UINT16_T, 1,  READ},
    {REG_ERROR_COUNT,       &registerErrorCounter,        UINT16_T, 3,  READ},
    {REG_ERROR_STATUS,      &registerErrorStatus,         UINT8_T,  1,  READ}
};


/**
 * @brief Find the index of the register in the constant register array.
 * @param regAddress The register address for which the index must be determined
 * @return The index of the provided register
 */
int8_t findRegIndex(uint8_t regAddress)
{
  uint8_t index = 0;
  uint8_t size = sizeof(registers) / sizeof(registers[0]);

  while((index < size) && (registers[index].adres != regAddress)) ++index;

  if(index == size)
    registerErrorStatus = ADDRESS_ERROR;

  return (index == size ? -1 : index);
}

/**
 * @brief Writes data to a register
 * @param data The data to write to the register
 * @param lenght The size of the data to write to the register
 */
void writeRegister(uint8_t *data, size_t lenght)
{
  int8_t regIndex = findRegIndex(data[0]);

  // Check if writing to register is allowed
  if(registers[regIndex].RW == READWRITE)
  {
    // Check the CRC of the message
    if(calculateCRC_CCITT(data, lenght) == 0)
    {
      if(registers[regIndex].datatype == UINT8_T)
      {
        uint8_t writeData = data[1];
        memcpy(registers[regIndex].regPtr, &writeData, sizeof(uint8_t));
      }
      else if(registers[regIndex].datatype == UINT16_T)
      {
        // convert the 2 uint8_t byte array to a uint16_t
        uint16_t writeData = ((data[2]<<8) & 0xFF00) + (data[1] & 0xFF);
        memcpy(registers[regIndex].regPtr, &writeData, sizeof(uint16_t));
      }
    }
    // Message CRC is not correct. CRC error occurred
    else
      registerErrorStatus = CRC_ERROR;
  }
  // Register is read only. Write error occurred
  else
    registerErrorStatus = WRITE_ERROR;
}

void readRegister(uint8_t regIndex, uint8_t *data, uint8_t size)
{
  memcpy(data, registers[regIndex].regPtr, size);
}

uint8_t readMeasStart(void)
{
  return registerMeasurementStart;
}

uint16_t readMeasSamples(void)
{
  return registerMeasurementSamples;
}

void storeMeasurement(uint16_t data, uint8_t sensor)
{
  registerMeasurementData[sensor] = data;
}

void setMeasurementStatus(MeasurementStatus status)
{
  registerMeasurementStatus = status;
}
