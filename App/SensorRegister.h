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
#define DEF_FIRMWARE_VERSION  "x.y"
#define DEF_PROTOCOL_VERSION  0x00
#define DEF_SENSOR_TYPE       0x01
#define DEF_MEAS_START        0x00
#define DEF_MEAS_STATUS       0x00
#define DEF_MEAS_TIME         0x0064
#define DEF_MEAS_SIZE         0x04
#define DEF_MEAS_DATA         0xAFFF
#define DEF_SENSOR_AMOUNT     0x02
#define DEF_SENSOR_SELECTED   0x00
#define DEF_MEAS_TYPE         0x00
#define DEF_MEAS_SAMPLES      0x000A
#define DEF_SENSOR_UNIT       0x02
#define DEF_SENSOR_SIZE       0x02
#define DEF_SENSOR_DATA       0xFFFF

/* Typedefs */
typedef enum{
  UINT8_T = 1,
  UINT16_T = 2
}tENUM_Datatype;

typedef enum{
  READ,
  READWRITE
}tENUM_READWRITE;

typedef struct
{
    uint8_t adres;
    void *regPtr;
    tENUM_Datatype datatype;
    uint8_t size;
    tENUM_READWRITE RW;
}SensorReg;

/* Register Declaration */
static uint8_t registerFirmwareVersion[10];
static uint8_t registerProtocolVersion = DEF_PROTOCOL_VERSION;
static uint16_t registerSensorType = DEF_SENSOR_TYPE;
static uint8_t registerMeasurementStart = DEF_MEAS_START;
static uint8_t registerMeasurementStatus = DEF_MEAS_STATUS;
static uint16_t registerMeasurementTime = DEF_MEAS_TIME;
static uint8_t registerMeasurementSize = DEF_MEAS_SIZE;
static uint16_t registerMeasurementData[2] = {DEF_MEAS_DATA, DEF_MEAS_DATA};
static uint8_t registerSensorAmount = DEF_SENSOR_AMOUNT;
static uint8_t registerSensorSelected = DEF_SENSOR_SELECTED;
static uint8_t registerMeasurementType = DEF_MEAS_TYPE;
static uint16_t registerMeasurementSamples = DEF_MEAS_SAMPLES;
static uint8_t registerSensorUnit = DEF_SENSOR_UNIT;
static uint8_t registerSensorSize = DEF_SENSOR_SIZE;
static uint16_t registerSensorData = DEF_SENSOR_DATA;


static const SensorReg registers[] =
{
    {REG_FIRMWARE_VERSION,  &registerFirmwareVersion,     UINT8_T,  10, READ},
    {REG_PROTOCOL_VERSION,  &registerProtocolVersion,     UINT8_T,  1,  READ},
    {REG_SENSOR_TYPE,       &registerSensorType,          UINT16_T, 1,  READ},
    {REG_MEAS_START,        &registerMeasurementStart,    UINT8_T,  1,  READWRITE},
    {REG_MEAS_STATUS,       &registerMeasurementStatus,   UINT8_T,  1,  READ},
    {REG_MEAS_TIME,         &registerMeasurementTime,     UINT16_T, 1,  READWRITE},
    {REG_MEAS_SIZE,         &registerMeasurementSize,     UINT8_T,  1,  READ},
    {REG_MEAS_DATA,         &registerMeasurementData,     UINT16_T, 2,  READ},
    {REG_SENSOR_AMOUNT,     &registerSensorAmount,        UINT8_T,  1,  READ},
    {REG_SENSOR_SELECTED,   &registerSensorSelected,      UINT8_T,  1,  READWRITE},
    {REG_MEAS_TYPE,         &registerMeasurementType,     UINT8_T,  1,  READWRITE},
    {REG_MEAS_SAMPLES,      &registerMeasurementSamples,  UINT16_T, 1,  READWRITE},
    {REG_SENSOR_UNIT,       &registerSensorUnit,          UINT8_T,  1,  READ},
    {REG_SENSOR_SIZE,       &registerSensorSize,          UINT8_T,  1,  READ},
    {REG_SENSOR_DATA,       &registerSensorData,          UINT16_T, 1,  READ}
};

/* Functions */

int8_t findRegIndex(uint8_t regAddress);
//void writeRegister(uint8_t *data, size_t lenght);
//void readRegister(uint8_t *data, size_t lenght);


#endif /* SENSORREGISTER_H_ */
