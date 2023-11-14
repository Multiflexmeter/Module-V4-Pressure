
#include "main.h"
#include "Huba.h"
#include <string.h>

uint8_t hubaBuffer[32];
uint8_t timeBuffer[32];
uint32_t strobeTimeStart = 0;
uint32_t strobeTimeEnd = 0;
uint8_t bitIndex = 0;
bool firstCapture = false;
bool hubaDone = false;

void hubaInit(TIM_HandleTypeDef *htim)
{
  bitIndex = 0;
  memset(timeBuffer, 0, 32);
  HAL_Delay(2);
  HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
}

SensorData hubaBufferToData(void)
{
  SensorData sensorData;
  uint8_t dataBuffer[3] = {0, 0, 0};
  uint8_t byteIndex = 0;
  uint8_t bitIndex = 7;

  /* Convert the time buffer to binary data */
  for(uint8_t i=1; i<30; i++)
  {
    /* Start bit */
    if(hubaBuffer[i]>12 && hubaBuffer[i]<20)
    {
      byteIndex++;
      bitIndex = 7;
    }

    /* Low bit */
    else if(hubaBuffer[i]>20 && hubaBuffer[i]<28)
    {
      bitIndex--;
    }

    /* High bit */
    else if(hubaBuffer[i]>4 && hubaBuffer[i]<12)
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

void hubaTimerCallback(TIM_HandleTypeDef *htim)
{
  /* Capture the falling edge */
  if(!firstCapture)
  {
    /* Store the start time */
    strobeTimeStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    firstCapture = true;
  }

  /* Capture the rising edge */
  else
  {
    uint32_t difference = 0;

    /* Store the end time */
    strobeTimeEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    /* Determine the difference */
    if(strobeTimeEnd > strobeTimeStart)
      difference = strobeTimeEnd - strobeTimeStart;

    else if(strobeTimeStart > strobeTimeEnd)
      difference = (0xFFFFFFFF - strobeTimeStart) + strobeTimeEnd;

    /* Reset if the time if larger then 150us */
    if(difference > 5000)
    {
      bitIndex = 0;
    }
    else
    {
      timeBuffer[bitIndex] = difference/32;

      if(bitIndex >= TIMEBUFFER_SIZE - 1)
      {
        bitIndex = 0;
        memcpy(hubaBuffer, timeBuffer, TIMEBUFFER_SIZE);
        hubaDone = true;
      }
      else
        bitIndex++;
    }

    __HAL_TIM_SET_COUNTER(htim, 0);
    firstCapture = false;
  }
}
