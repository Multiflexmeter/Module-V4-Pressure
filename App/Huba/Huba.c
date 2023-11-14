
#include "main.h"
#include "Huba.h"
#include <stdbool.h>


SensorData hubaBufferToData(const uint8_t *buffer)
{
  SensorData sensorData;
  uint8_t dataBuffer[3] = {0, 0, 0};
  uint8_t byteIndex = 0;
  uint8_t bitIndex = 7;

  /* Convert the time buffer to binary data */
  for(uint8_t i=1; i<30; i++)
  {
    /* Start bit */
    if(buffer[i]>12 && buffer[i]<20)
    {
      byteIndex++;
      bitIndex = 7;
    }

    /* Low bit */
    else if(buffer[i]>20 && buffer[i]<28)
    {
      bitIndex--;
    }

    /* High bit */
    else if(buffer[i]>4 && buffer[i]<12)
    {
      dataBuffer[byteIndex] |= 1<<bitIndex;
      bitIndex--;
    }
  }

  /* Convert data to pressure and temperature */
  sensorData.pressure = (float) ((((dataBuffer[0]<<8) + dataBuffer[1])-3000)/8000.0) * 0.6; // Pressure in bar
  sensorData.temperature = (float) ((dataBuffer[2]*200.0)/255) - 50; // temp in celsius

  return sensorData;
}
