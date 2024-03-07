
#include "main.h"
#include "Huba.h"
#include <string.h>

void hubaStart(HubaSensor *sensor)
{
  sensor->bitIndex = 0;
  memset(sensor->tmpBuf, 0, 32);
  HAL_Delay(2);
  HAL_TIM_IC_Start_IT(sensor->htim, TIM_CHANNEL_1);
}

SensorDataHuba hubaBufferToData(HubaSensor *sensor)
{
  SensorDataHuba sensorData;
  uint8_t dataBuffer[3] = {0, 0, 0};
  uint8_t byteIndex = 0;
  uint8_t bitIndex = 7;

  /* Convert the time buffer to binary data */
  for(uint8_t i=1; i<30; i++)
  {
    /* Start bit */
    if(sensor->hubaBuf[i]>12 && sensor->hubaBuf[i]<20)
    {
      byteIndex++;
      bitIndex = 7;
    }

    /* Low bit */
    else if(sensor->hubaBuf[i]>20 && sensor->hubaBuf[i]<28)
    {
      bitIndex--;
    }

    /* High bit */
    else if(sensor->hubaBuf[i]>4 && sensor->hubaBuf[i]<12)
    {
      dataBuffer[byteIndex] |= 1<<bitIndex;
      bitIndex--;
    }
  }

  /* copy data to pressure and temperature */
  sensorData.pressureData = (dataBuffer[0]<<8) + dataBuffer[1];
  sensorData.temperatureData = dataBuffer[2];

  return sensorData;
}

void hubaTimerCallback(HubaSensor *sensor)
{
  /* Capture the falling edge */
  if(!sensor->firstCapture)
  {
    /* Store the start time */
    sensor->strobeTimeStart = HAL_TIM_ReadCapturedValue(sensor->htim, TIM_CHANNEL_1);
    sensor->firstCapture = true;
  }

  /* Capture the rising edge */
  else
  {
    uint32_t difference = 0;

    /* Store the end time */
    sensor->strobeTimeEnd = HAL_TIM_ReadCapturedValue(sensor->htim, TIM_CHANNEL_1);

    /* Determine the difference */
    if(sensor->strobeTimeEnd > sensor->strobeTimeStart)
      difference = sensor->strobeTimeEnd - sensor->strobeTimeStart;

    else if(sensor->strobeTimeStart > sensor->strobeTimeEnd)
      difference = (0xFFFFFFFF - sensor->strobeTimeStart) + sensor->strobeTimeEnd;

    /* Reset if the time if larger then 150us */
    if(difference > 5000)
    {
      sensor->bitIndex = 0;
    }
    else
    {
      sensor->tmpBuf[sensor->bitIndex] = difference/32;

      if(sensor->bitIndex >= TIMEBUFFER_SIZE - 1)
      {
        sensor->bitIndex = 0;
        memcpy(sensor->hubaBuf, sensor->tmpBuf, TIMEBUFFER_SIZE);
        sensor->hubaDone = true;
      }
      else
        sensor->bitIndex++;
    }

    __HAL_TIM_SET_COUNTER(sensor->htim, 0);
    sensor->firstCapture = false;
  }
}
