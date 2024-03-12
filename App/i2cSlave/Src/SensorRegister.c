/**
  ******************************************************************************
  * @file           SensorRegister.c
  * @brief          SensorRegister functions
  * @author         D.Kerstens
  ******************************************************************************
  */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "SensorRegister.h"
#include "crc16.h"

/* Register Declaration */
static uint8_t registerFirmwareVersion[10] = DEF_FIRMWARE_VERSION;
static uint8_t registerProtocolVersion = DEF_PROTOCOL_VERSION;
static uint16_t registerSensorType = DEF_SENSOR_TYPE;
static uint8_t registerInitStart= DEF_INIT_START;
static uint8_t registerInitStatus= DEF_INIT_STATUS;
static uint8_t registerMeasurementStart = DEF_MEAS_START;
static uint8_t registerMeasurementStatus = DEF_MEAS_STATUS;
static uint16_t registerMeasurementTime = DEF_MEAS_TIME;
static SensorData registerMeasurementDataKeller[2] = {{DEF_MEAS_DATA, DEF_MEAS_DATA}};
static SensorDataHuba registerMeasurementDataHuba[2] = {{0xFFFF, 0xFF}};
static uint8_t registerSensorAmount = DEF_SENSOR_AMOUNT;
static uint8_t registerSensorSelected = DEF_SENSOR_SELECTED;
static uint8_t registerMeasurementType = DEF_MEAS_TYPE;
static uint8_t registerMeasurementSamples = DEF_MEAS_SAMPLES;
static SensorData registerSensorDataKeller = {DEF_MEAS_DATA, DEF_MEAS_DATA};
static SensorDataHuba registerSensorDataHuba = {0xFFFF, 0xFF};
static uint16_t registerErrorCounter[3] = {DEF_ERROR_COUNT, DEF_ERROR_COUNT, DEF_ERROR_COUNT};
static uint8_t registerErrorStatus = DEF_ERROR_STATUS;

void updateMeasureTime(void);

SensorReg registers[] =
{
    {REG_FIRMWARE_VERSION,  &registerFirmwareVersion,     UINT8_T,  10, READ,       0},
    {REG_PROTOCOL_VERSION,  &registerProtocolVersion,     UINT8_T,  1,  READ,       0},
    {REG_SENSOR_TYPE,       &registerSensorType,          UINT16_T, 1,  READ,       0},
    {REG_INIT_START,        &registerInitStart,           UINT8_T,  1,  READWRITE,  0},
    {REG_INIT_STATUS,       &registerInitStatus,          UINT8_T,  1,  READ,       0},
    {REG_MEAS_START,        &registerMeasurementStart,    UINT8_T,  1,  READWRITE,  0},
    {REG_MEAS_STATUS,       &registerMeasurementStatus,   UINT8_T,  1,  READ,       0},
    {REG_MEAS_TIME,         &registerMeasurementTime,     UINT16_T, 1,  READWRITE,  0},
    {REG_MEAS_DATA,         &registerMeasurementDataKeller,     SENSORDATA_KELLER,  2,  READ,    0},
    {REG_SENSOR_AMOUNT,     &registerSensorAmount,        UINT8_T,  1,  READ,       0},
    {REG_SENSOR_SELECTED,   &registerSensorSelected,      UINT8_T,  1,  READWRITE,  0},
    {REG_MEAS_TYPE,         &registerMeasurementType,     UINT8_T,  1,  READWRITE,  0},
    {REG_MEAS_SAMPLES,      &registerMeasurementSamples,  UINT8_T,  1,  READWRITE,  updateMeasureTime},
    {REG_SENSOR_DATA,       &registerSensorDataKeller,          SENSORDATA_KELLER, 1,  READ,     0},
    {REG_ERROR_COUNT,       &registerErrorCounter,        UINT16_T, 3,  READ,       0},
    {REG_ERROR_STATUS,      &registerErrorStatus,         UINT8_T,  1,  READ,       0},
};

/**
 * @fn const bool invalidIndex(int8_t)
 * @brief functions return true if index is not within the valid range
 *
 * @param index
 * @return
 */
const bool invalidIndex( int8_t index )
{
  return (index < 0 || index >= sizeof(registers) / sizeof(registers[0])) ? true : false;
}

/**
 * @brief Find the index of the register in the constant register array.
 *
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
 *
 * @param data The data to write to the register
 * @param lenght The size of the data to write to the register
 */
void writeRegister(uint8_t *data, size_t lenght)
{
  int8_t regIndex = findRegIndex(data[0]);

  if( regIndex < 0 )
  {
    registerErrorStatus = ADDRESS_ERROR;
    return;
  }

  /* Check if writing to register is allowed */
  if(registers[regIndex].RW == READWRITE)
  {
    /* Check the CRC of the message */
    if(calculateCRC_CCITT(data, lenght) == 0)
    {
      if(registers[regIndex].datatype == UINT8_T)
      {
        uint8_t writeData = data[1];
        memcpy(registers[regIndex].regPtr, &writeData, sizeof(uint8_t));
      }
      else if(registers[regIndex].datatype == UINT16_T)
      {
        /* convert the 2 uint8_t byte array to a uint16_t */
        uint16_t writeData = ((data[2]<<8) & 0xFF00) + (data[1] & 0xFF);
        memcpy(registers[regIndex].regPtr, &writeData, sizeof(uint16_t));
      }

      //check if a change callback is availble, then call it.
      if( registers[regIndex].changeCallback )
      {
        registers[regIndex].changeCallback();
      }
    }
    /* Message CRC is not correct. CRC error occurred */
    else
      registerErrorStatus = CRC_ERROR;
  }
  /* Register is read only. Write error occurred */
  else
    registerErrorStatus = WRITE_ERROR;
}

/**
 * @brief Reads the register and copies the data to a buffer
 *
 * @param regIndex is the register index to copy from
 * @param data is the data buffer to copy the data to
 * @param size is the size of the data
 */
void readRegister(uint8_t regIndex, uint8_t *data, uint8_t size)
{
  memcpy(data, registers[regIndex].regPtr, size);
}

/**
 * @fn SensorType readSensorType(void)
 * @brief function to return the current sensorTyp value
 *
 * @return \ref SensorType
 */
SensorType readSensorType(void)
{
  return registerSensorType;
}

/**
 * @fn void setSensorType(SensorType)
 * @brief function to set the current sensorTyp
 *
 * @param type \ref SensorType
 */
void setSensorType(SensorType type)
{
  registerSensorType = type;
}

/**
 * @brief Reads the measurement start register
 *
 * @return Returns 1 if the measurement is started
 *         Returns 0 if the measurements is stopped
 */
uint8_t readMeasStart(void)
{
  return registerMeasurementStart;
}

/**
 * @fn void stopMeas(void)
 * @brief function to stop the current measurement
 *
 */
void stopMeas(void)
{
  registerMeasurementStart = 0;
}

/**
 * @brief Reads the amount of samples the measurement must have
 *
 * @return returns the amount of samples
 */
uint8_t readMeasSamples(void)
{
  return registerMeasurementSamples;
}

/**
 * @fn void storeMeasurementKeller(float, float, uint8_t)
 * @brief Stores the measurement in the designated register
 *
 * @param pressure : measured pressure
 * @param temperature : measured temperature in degree celsius
 * @param sensor : number of sensor 0 or 1
 */
void storeMeasurementKeller(float pressure, float temperature, uint8_t sensor)
{
  if( sensor >= DEF_SENSOR_AMOUNT ) //validate sensor index
    return;
  registerMeasurementDataKeller[sensor].pressureData = pressure;
  registerMeasurementDataKeller[sensor].temperatureData = temperature;
}

/**
 * @fn void storeMeasurementHuba(uint8_t, uint8_t, uint8_t, uint8_t)
 * @brief stores the huba measurement data.
 *
 * @param pressureHighByte  HighByte of pressure: The digital pressure output signal is a 14 bit value. The digital value 3000 represents the 0% FSO point and the digital value 11000 represents the 100%
 * @param pressureLowByte   LowByte of pressure: The digital pressure output signal is a 14 bit value. The digital value 3000 represents the 0% FSO point and the digital value 11000 represents the 100%
 * @param temperature temperature byte : an 8-bit temperature quantity spanning from -50 to 150°C.
 * @param sensor is the sensor the data is taken from
 */
void storeMeasurementHuba(uint16_t pressure, uint8_t temperature, uint8_t sensor)
{
  if( sensor >= DEF_SENSOR_AMOUNT ) //validate sensor index
    return;
  registerMeasurementDataHuba[sensor].pressureData = pressure;
  registerMeasurementDataHuba[sensor].temperatureData = temperature;
}

/**
 * @fn void clearMeasurement(uint8_t)
 * @brief clear the current measurement to 0xFF value
 *
 * @param sensor is the sensor the data is taken from
 */
void clearMeasurement( uint8_t sensor)
{
  if( sensor >= DEF_SENSOR_AMOUNT ) //validate sensor index
    return;

  memset(&registerMeasurementDataKeller[sensor], 0xFF, sizeof(registerMeasurementDataKeller[sensor]));
  memset(&registerMeasurementDataHuba[sensor], 0xFF, sizeof(registerMeasurementDataHuba[sensor]));
}

/**
 * @brief Set the measurement status
 *
 * @param status is the status to set
 */
void setMeasurementStatus(MeasurementStatus status)
{
  registerMeasurementStatus = status;
}

/**
 * @fn void storeSelectedSensor(uint8_t)
 * @brief function to store the selected sensor data
 *
 * @param sensor 0 or 1
 */
void storeSelectedSensor(uint8_t sensor)
{
  /* Ignore invalid sensor numbers */
  if(sensor > (registerSensorAmount-1))
    return;

  /* Store selected sensor data */
  registerSensorDataKeller = registerMeasurementDataKeller[sensor];
  registerSensorDataHuba = registerMeasurementDataHuba[sensor];
}

/**
 * @fn void enableInitFunction(void)
 * @brief function to enable runtime the init function
 *
 */
const void enableInitFunction(void)
{
  //check if function is disabled, then enable it.
  if( registerInitStatus == 0xFF)
  {
    registerInitStatus = 0x00; //enable init function
  }
}

/**
 * @fn bool getInitStartStatus(void)
 * @brief function to get the sensor init start status
 *
 * @return true = start init, false = no init
 */
const bool getInitStartStatus(void)
{
  return registerInitStart;
}

/**
 * @fn void setInitStatusBusy(void)
 * @brief function to set init status busy
 *
 */
const void setInitStatusBusy(void)
{
  registerInitStatus = 0x01; //set busy
}

/**
 * @fn void setInitStatusReady(void)
 * @brief function to set init status ready
 *
 */
const void setInitStatusReady(bool resultOkay)
{
  registerInitStatus = resultOkay ? 0x0A : 0x0F; //set ready 0x0A or failure 0x0F
  registerInitStart = 0; //reset
}

/**
 * @fn const void setMeasureTime(uint16_t)
 * @brief function to set a conditional measure time based on sensormodule type
 * Minimum value of \ref DEF_MEAS_TIME is used
 *
 * @param newTime
 */
const void setMeasureTime(uint16_t newTime)
{
  if( newTime > DEF_MEAS_TIME )
  {
    registerMeasurementTime = newTime;
  }
}

/**
 * @fn const uint8_t getSelectedSensor(void)
 * @brief function to return the selected sensor
 *
 * @return sensor number, starting from 0.
 */
const uint8_t getSelectedSensor(void)
{
  return registerSensorSelected;
}

/**
 * @fn void updateMeasureTime(void)
 * @brief update routine for measure time for a new number of samples
 *
 */
void updateMeasureTime(void)
{
  uint16_t samples = registerMeasurementSamples;
  uint16_t newTime = 0;
  if( samples == 0 )
    samples = 1;
  if( samples > 100 )
    samples = 100;

  switch(registerSensorType)
  {
    case MFM_DRUKMODULE_RS485:

      newTime = DEF_MEAS_TIME_START_RS485 + samples * DEF_MEAS_TIME_PER_SAMPLE_RS485;
      registerMeasurementTime = newTime;

      break;

    case MFM_DRUKMODULE_ONEWIRE:

      newTime = DEF_MEAS_TIME_START_ONEWIRE + samples * DEF_MEAS_TIME_PER_SAMPLE_ONEWIRE;
      registerMeasurementTime = newTime;

      break;

    default:

      registerMeasurementTime = DEF_MEAS_TIME;

      break;
  }
}

/**
 * @fn const void setMeasureDataSize(SensorType)
 * @brief function to change measure data size
 *
 * @param type sensortype
 */
const void setMeasureDataSize(SensorType type)
{
  if (type != MFM_DRUKMODULE_RS485 && type != MFM_DRUKMODULE_ONEWIRE)
    return;

  uint8_t indexMeas = findRegIndex(REG_MEAS_DATA); //find register index
  uint8_t indexSensor = findRegIndex(REG_SENSOR_DATA); //find register index

  if (type == MFM_DRUKMODULE_RS485)
  {
    registers[indexMeas].datatype = SENSORDATA_KELLER;
    registers[indexMeas].regPtr = &registerMeasurementDataKeller;
    registers[indexSensor].datatype = SENSORDATA_KELLER;
    registers[indexSensor].regPtr = &registerSensorDataKeller;

  }
  else if (type == MFM_DRUKMODULE_ONEWIRE)
  {
    registers[indexMeas].datatype = SENSORDATA_HUBA;
    registers[indexMeas].regPtr = &registerMeasurementDataHuba;
    registers[indexSensor].datatype = SENSORDATA_HUBA;
    registers[indexSensor].regPtr = &registerSensorDataHuba;
  }
}
