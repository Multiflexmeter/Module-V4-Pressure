/*
 * SensorRegister.c
 *
 *  Created on: 12 jul. 2023
 *      Author: danny.kerstens
 */

#include <stdint.h>
#include "SensorRegister.h"


uint8_t findRegIndex(uint8_t regAddress)
{
  uint8_t index = 0;
  uint8_t size = sizeof(registers) / sizeof(registers[0]);

  while((index < size) && (registers[index].adres != regAddress)) ++index;

  return (index == size ? -1 : index);
}
