/*
 * KellerModbus.h
 *
 *  Created on: 5 sep. 2023
 *      Author: danny.kerstens
 */

#ifndef KELLER_KELLERMODBUS_H_
#define KELLER_KELLERMODBUS_H_

/* Modbus Addresses */
#define BROADCAST_ADDRESS         0x00
#define DEFAULT_ADDRESS           0x01

/* Modbus Functions */
#define REGISTER_READ             0x03
#define SINGLE_WRITE              0x06
#define ECHO_TEST                 0x08
#define REGISTER_WRITE            0x10

/* Keller Registers */
#define CALCULATED_VALUE          0x0000
#define PRESSURE_SENSOR1          0x0002
#define PRESSURE_SENSOR2          0x0004
#define TEMPERATURE               0x0006
#define TEMPERATURE_SENSOR1       0x0006
#define TEMPERATURE_SENSOR2       0x0006


#endif /* KELLER_KELLERMODBUS_H_ */
