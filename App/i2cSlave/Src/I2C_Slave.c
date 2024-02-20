
#include <string.h>
#include <stdbool.h>
#include "I2C_Slave.h"
#include "SensorRegister.h"
#include "crc16.h"

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  5
#define CRC_SIZE 2

uint8_t RxData[RxSIZE];
uint8_t regWriteData[RxSIZE];
uint8_t txBuffer[40]; //maximum needed for transmit measuring data: 1 byte datasize + 36 bytes data + 2 bytes crc: 39

int8_t regIndex;
uint8_t regSize;
bool writeFlag = false;
int8_t rxcount = 0;

volatile bool sensorSlaveErrorFlag = false;

/* Functions */


/**
 * @brief Calculate the CRC and transmit the sensor register data
 * @param data The register data to be transmitted
 * @param size The size of the data
 */
void sensorSlaveTransmit(uint8_t *data, uint8_t size)
{
  // Calculate the crc of the message
  uint16_t crc = calculateCRC_CCITT(data, size);
  data[size+1] = crc & 0xFF;
  data[size] = crc >> 8 & 0xFF;

  // Transmit the data + crc over the i2c bus
  HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, data, size+CRC_SIZE, I2C_FIRST_AND_LAST_FRAME);
}

/* Callbacks */
void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
  // Re-enable the listen interrupt after it has been triggered
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  // The master wants to transmit data
  if (TransferDirection == I2C_DIRECTION_TRANSMIT)
  {
    // Receive the sensor register adres
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData, 1, I2C_FIRST_FRAME);
    rxcount = 0;
  }

  // Master requests to read the selected register
  else
  {
    if( invalidIndex(regIndex) ) //wrong register
    {
      __HAL_I2C_DISABLE(hi2c); //abort with disable, prevent stalling SCL low.

      sensorSlaveErrorFlag = true; //set flag to reboot I2C at main

      return;
    }


    // Transmit the data in the selected register
    if(registers[regIndex].adres == REG_MEAS_DATA || registers[regIndex].adres == REG_SENSOR_DATA)
    {
      txBuffer[0] = regSize;
      readRegister(regIndex, txBuffer+1, regSize);
      sensorSlaveTransmit(txBuffer, regSize+1);
    }
    else
    {
      readRegister(regIndex, txBuffer, regSize);
      sensorSlaveTransmit(txBuffer, regSize);
    }
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // Find the register and store the size
  if(rxcount == 0)
  {
    regIndex = findRegIndex(RxData[0]);

    // Abort if the register is not found.
    if( invalidIndex(regIndex) )
    {
      return;
    }

    regSize = registers[regIndex].datatype * registers[regIndex].size;

    // Abort if the register size is wrong
    if(regSize < 0)
    {
      return;
    }
  }

  // Receive the data and CRC
  rxcount++;
  if(rxcount <= regSize + CRC_SIZE)
  {

    if (rxcount == regSize + CRC_SIZE)
    {
      // Receive the last frame of the message
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
    else
    {
      // Receive the next frame of the message
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
    }
  }

  if(rxcount > regSize + CRC_SIZE)
  {
    // Handle writing to the register in the main loop
    rxcount = 0;
    memcpy(regWriteData, RxData, regSize + CRC_SIZE + 1);
    writeFlag = true;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}
