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

/* Keller Functions */
#define READ_COEFFICIENTS         0x1E  //F30: Read coefficients
#define WRITE_COEFFICIENTS        0x1F  //F31: Write coefficients
#define READ_CONFIGURATION        0x20  //F32: Read configuration
#define WRITE_CONFIGURATION       0x21  //F33: Write configuration
#define INITIALIZE                0x30  //F48: Initialize and release
#define NEW_ADDRESS               0x42  //F66: Write and read new device address
#define READ_SERIALNUMBER         0x45  //F69: Read the serial number
#define READ_CHANNEL_FLOAT        0x49  //F73: Read value of a channel float
#define READ_CHANNEL_LONG         0x4A  //F74: Read value of a channel 32bit integer

/* Keller Errors */
#define INITIALISATION_ERROR      0x20

/* Keller Registers */
#define CALCULATED_VALUE          0x0000
#define PRESSURE_SENSOR1          0x0002
#define PRESSURE_SENSOR2          0x0004
#define TEMPERATURE               0x0006
#define TEMPERATURE_SENSOR1       0x0008
#define TEMPERATURE_SENSOR2       0x000A
#define UART_CONFIGURATION        0x0200


#endif /* KELLER_KELLERMODBUS_H_ */
