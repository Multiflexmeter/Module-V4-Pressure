/*
 * I2C_Slave.c
 *
 *  Created on: Jul 11, 2023
 *      Author: danny.kerstens
 */
#include <string.h>
#include "I2C_Slave.h"
#include "SensorRegister.h"

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  3
uint8_t RxData[RxSIZE];
uint8_t RegAdres;
uint8_t txBuffer[10];
int8_t regIndex;
uint8_t regSize;
int rxcount = 0;

/* Callbacks */
void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  // Receive the sensor register adres
  if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
  {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &RegAdres, 1, I2C_FIRST_FRAME);
  }

  // Transmit the data in the selected register
  else if (TransferDirection == I2C_DIRECTION_RECEIVE)
  {
    memcpy(txBuffer, registers[regIndex].regPtr, regSize);
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, regSize, I2C_FIRST_AND_LAST_FRAME);
  }
  rxcount = 0;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(rxcount == 0)
  {
    regIndex = findRegIndex(RegAdres);
    regSize = registers[regIndex].datatype * registers[regIndex].size;
    if(regSize < 0)
      return;
  }

  if(rxcount < regSize)
  {
    if (rxcount == regSize-1)
    {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
    else
    {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
    }
  }

  if(rxcount >= regSize)
  {
    rxcount = 0;
    *((uint8_t *) registers[regIndex].regPtr) = RxData[0];
    // Process data
  }
  rxcount++;
}

//void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//
//}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}
