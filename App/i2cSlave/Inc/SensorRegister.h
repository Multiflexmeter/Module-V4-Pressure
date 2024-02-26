
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
#define DEF_FIRMWARE_VERSION  "0.3d"
#define DEF_PROTOCOL_VERSION  0x00
#define DEF_SENSOR_TYPE       0x01
#define DEF_INIT_START        0x00
#define DEF_INIT_STATUS       0xFF //default not available, when be enabled runtime.
#define DEF_MEAS_START        0x00
#define DEF_MEAS_STATUS       0x00
#define DEF_MEAS_TIME         0x0064
#define DEF_MEAS_DATA         0xFFFFFFFF
#define DEF_SENSOR_AMOUNT     0x02
#define DEF_SENSOR_SELECTED   0x00
#define DEF_MEAS_TYPE         0x00
#define DEF_MEAS_SAMPLES      0x0A
#define DEF_SENSOR_DATA       0xFFFF
#define DEF_ERROR_COUNT       0x0000
#define DEF_ERROR_STATUS      0

/* Typedefs */
typedef enum{
  UINT8_T = 1,
  UINT16_T = 2,
  SENSORDATA = 8
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

typedef struct __attribute__((__packed__))
{
  float pressure;
  float temperature;
}SensorData;

typedef struct
{
  uint8_t adres;
  void *regPtr;
  tENUM_Datatype datatype;
  uint8_t size;
  tENUM_READWRITE RW;
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
void setMeasurementStatus(MeasurementStatus status);
void storeSelectedSensor(uint8_t sensor);

const void enableInitFunction(void);
const bool getInitStartStatus(void);
const void setInitStatusBusy(void);
const void setInitStatusReady(bool resultOkay);

#endif /* SENSORREGISTER_H_ */
