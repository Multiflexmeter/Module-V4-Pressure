
#ifndef SENSORREGISTER_H_
#define SENSORREGISTER_H_

/* Sensor Registers */
#define REG_FIRMWARE_VERSION      0x01
#define REG_PROTOCOL_VERSION      0x02
#define REG_SENSOR_TYPE           0x03
#define REG_INIT_START            0x0A
#define REG_INIT_STATUS           0x0B
#define REG_MEAS_START            0x10
#define REG_MEAS_STATUS           0x11
#define REG_MEAS_TIME             0x12
#define REG_MEAS_DATA             0x20
#define REG_SENSOR_AMOUNT         0x30
#define REG_SENSOR_SELECTED       0x31
#define REG_MEAS_TYPE             0x32
#define REG_MEAS_SAMPLES          0x33
#define REG_SENSOR_DATA           0x38
#define REG_ERROR_COUNT           0x50
#define REG_ERROR_STATUS          0x51

/* Registers Defaults*/
#define DEF_FIRMWARE_VERSION  "0.5b"
#define DEF_PROTOCOL_VERSION  0x00
#define DEF_SENSOR_TYPE       0x01
#define DEF_INIT_START        0x00
#define DEF_INIT_STATUS       0xFF //default not available, when be enabled runtime.
#define DEF_MEAS_START        0x00
#define DEF_MEAS_STATUS       0x00
#define DEF_MEAS_TIME         0x0064 //100ms for 10 samples
#define DEF_MEAS_TIME_ONEWIRE (DEF_MEAS_TIME) //for 10 samples
#define DEF_MEAS_TIME_START_ONEWIRE       0x13 //19
#define DEF_MEAS_TIME_PER_SAMPLE_ONEWIRE  0x05 //5
#define DEF_MEAS_TIME_RS485   0x00C8 //200ms for 10 samples
#define DEF_MEAS_TIME_START_RS485       0x3C //60
#define DEF_MEAS_TIME_PER_SAMPLE_RS485  0x10 //16
#define DEF_MEAS_DATA         0xFFFFFFFF
#define DEF_SENSOR_AMOUNT     0x02
#define DEF_SENSOR_SELECTED   0x00
#define DEF_MEAS_TYPE         0x00
#define DEF_MEAS_SAMPLES      0x0A
#define DEF_SENSOR_DATA       0xFFFF
#define DEF_ERROR_COUNT       0x0000
#define DEF_ERROR_STATUS      0

/* Typedefs */
typedef struct __attribute__((__packed__))
{
  float pressureData;
  float temperatureData;
}SensorData;

typedef struct __attribute__((__packed__))
{
  uint16_t pressureData;
  uint8_t temperatureData;
}SensorDataHuba;

typedef struct __attribute__((__packed__))
{
  SensorData Keller;
  SensorDataHuba Huba;
}Union_SensorData;

typedef enum{
  UINT8_T = 1,
  UINT16_T = 2,
  SENSORDATA = (sizeof(SensorData)),
  SENSORDATA_HUBA = (sizeof(SensorDataHuba)),
}tENUM_Datatype;

typedef enum{
  NO_ERROR,
  CRC_ERROR,
  ADDRESS_ERROR,
  WRITE_ERROR
}tENUM_Error;

typedef enum{
  READ,
  READWRITE
}tENUM_READWRITE;

typedef enum{
  MFM_DRUKMODULE_RS485 = 0x01,
  MFM_DRUKMODULE_ONEWIRE = 0x02
}SensorType;

typedef enum{
  NO_MEASUREMENT = 0x00,
  MEASUREMENT_ACTIVE = 0x01,
  MEASUREMENT_DONE = 0x0A,
  MEASUREMENT_ERROR = 0xF0
}MeasurementStatus;

typedef enum{
  SINGLE_SAMPLE  = 0x00,
  AVERAGE_SAMPLE = 0x10,
  MEDIAN_SAMPLE  = 0x20
}MeasurementType;

typedef struct
{
  uint8_t adres;
  void *regPtr;
  tENUM_Datatype datatype;
  uint8_t size;
  tENUM_READWRITE RW;
  void (* changeCallback)(void);
}SensorReg;

extern const SensorReg registers[];

/* Functions */
const bool invalidIndex( int8_t index );
int8_t findRegIndex(uint8_t regAddress);
void writeRegister(uint8_t *data, size_t lenght);
void readRegister(uint8_t regIndex, uint8_t *data, uint8_t size);

/* Internal register access functions */
SensorType readSensorType(void);
void setSensorType(SensorType type);
uint8_t readMeasStart(void);
void stopMeas(void);
uint8_t readMeasSamples(void);
void storeMeasurement(float pressure, float temperature, uint8_t sensor);
void storeMeasurementHuba(uint16_t pressure, uint8_t temperature, uint8_t sensor);
void clearMeasurement( uint8_t sensor);
void setMeasurementStatus(MeasurementStatus status);
void storeSelectedSensor(uint8_t sensor);

const void enableInitFunction(void);
const bool getInitStartStatus(void);
const void setInitStatusBusy(void);
const void setInitStatusReady(bool resultOkay);
const void setMeasureTime(uint16_t newTime);
const uint8_t getSelectedSensor(void);
const void setMeasureDataSize(SensorType type);

#endif /* SENSORREGISTER_H_ */
