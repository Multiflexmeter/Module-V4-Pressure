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
#define DEFAULT_MEAS_SIZE         4
#define DEFAULT_MEAS_DATA         0xFFFF
#define DEFAULT_SENSOR_AMOUNT     0x02
#define DEFAULT_SENSOR_SELECTED   0x00
#define DEFAULT_MEAS_TYPE         0x00
#define DEFAULT_MEAS_SAMPLES      0x000A
#define DEFAULT_SENSOR_UNIT       0x02
#define DEFAULT_SENSOR_SIZE       0x02
#define DEFAULT_SENSOR_DATA       0xFFFF

#endif /* SENSORREGISTER_H_ */
