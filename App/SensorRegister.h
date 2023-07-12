/*
 * SensorRegister.h
 *
 *  Created on: Jul 10, 2023
 *      Author: danny.kerstens
 */

#ifndef SENSORREGISTER_H_
#define SENSORREGISTER_H_

/* Sensor Registers */
#define REG_FIRMWARE_VERSION      0x01
#define REG_PROTOCOL_VERSION      0x02
#define REG_SENSOR_TYPE           0x03
#define REG_MEAS_START            0x10
#define REG_MEAS_STATUS           0x11
#define REG_MEAS_TIME             0x12
#define REG_MEAS_SIZE             0x20
#define REG_MEAS_DATA             0x21
#define REG_SENSOR_AMOUNT         0x30
#define REG_SENSOR_SELECTED       0x31
#define REG_MEAS_TYPE             0x32
#define REG_MEAS_SAMPLES          0x33
#define REG_SENSOR_UNIT           0x37
#define REG_SENSOR_SIZE           0x38
#define REG_SENSOR_DATA           0x39

/* Registers Defaults*/
#define DEFAULT_FIRMWARE_VERSION  "x.y"
#define DEFAULT_PROTOCOL_VERSION  0x00
#define DEFAULT_SENSOR_TYPE       0x01
#define DEFAULT_MEAS_START        0x00
#define DEFAULT_MEAS_STATUS       0x00
#define DEFAULT_MEAS_TIME         0x0064
#define DEFAULT_MEAS_SIZE         0x04
#define DEFAULT_MEAS_DATA         0xFFFF
#define DEFAULT_SENSOR_AMOUNT     0x02
#define DEFAULT_SENSOR_SELECTED   0x00
#define DEFAULT_MEAS_TYPE         0x00
#define DEFAULT_MEAS_SAMPLES      0x000A
#define DEFAULT_SENSOR_UNIT       0x02
#define DEFAULT_SENSOR_SIZE       0x02
#define DEFAULT_SENSOR_DATA       0xFFFF

/* Typedefs */
typedef enum{
  READ,
  READWRITE
}tENUM_READWRITE;

typedef struct
{
    uint8_t adres;
    void *regPtr;
    uint8_t size;
    tENUM_READWRITE RW;
}SensorReg;

/* Register Declaration */
static uint8_t registerFirmwareVersion[10];
static uint8_t registerProtocolVersion;
static uint16_t registerSensorType;
static uint8_t registerMeasurementStart;
static uint8_t registerMeasurementStatus;
static uint16_t registerMeasurementTime;
static uint8_t registerMeasurementSize;
static uint16_t registerMeasurementData[2];
static uint8_t registerSensorAmount;
static uint8_t registerSensorSelected;
static uint8_t registerMeasurementType;
static uint16_t registerMeasurementSamples;
static uint8_t registerSensorUnit;
static uint8_t registerSensorSize;
static uint16_t registerSensorData;


static const SensorReg registers[] =
{
    {REG_FIRMWARE_VERSION,  &registerFirmwareVersion,     10, READ},
    {REG_PROTOCOL_VERSION,  &registerProtocolVersion,     1,  READ},
    {REG_SENSOR_TYPE,       &registerSensorType,          2,  READ},
    {REG_MEAS_START,        &registerMeasurementStart,    1,  READWRITE},
    {REG_MEAS_STATUS,       &registerMeasurementStatus,   1,  READ},
    {REG_MEAS_TIME,         &registerMeasurementTime,     2,  READWRITE},
    {REG_MEAS_SIZE,         &registerMeasurementSize,     1,  READ},
    {REG_MEAS_DATA,         &registerMeasurementData,     4,  READ},
    {REG_SENSOR_AMOUNT,     &registerSensorAmount,        1,  READ},
    {REG_SENSOR_SELECTED,   &registerSensorSelected,      1,  READWRITE},
    {REG_MEAS_TYPE,         &registerMeasurementType,     1,  READWRITE},
    {REG_MEAS_SAMPLES,      &registerMeasurementSamples,  2,  READWRITE},
    {REG_SENSOR_UNIT,       &registerSensorUnit,          1,  READ},
    {REG_SENSOR_SIZE,       &registerSensorSize,          1,  READ},
    {REG_SENSOR_DATA,       &registerSensorData,          2,  READ}
};

/* Functions */

uint8_t findRegIndex(uint8_t regAddress);
void writeRegister(uint8_t *data, size_t lenght);
void readRegister(uint8_t *data, size_t lenght);


#endif /* SENSORREGISTER_H_ */
