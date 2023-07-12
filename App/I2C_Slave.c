/*
 * I2C_Slave.c
 *
 *  Created on: Jul 11, 2023
 *      Author: danny.kerstens
 */
#include "I2C_Slave.h"
#include "SensorRegister.h"

extern I2C_HandleTypeDef hi2c1;

#define RxSIZE  2
uint8_t RxData[RxSIZE];
uint8_t RegAdres;
uint8_t RegData = 0x31;
uint8_t SlaveDirection;
uint8_t regIndex;

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
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData, 1, I2C_FIRST_FRAME);
  }

  // Transmit the data in the selected register
  else if (TransferDirection == I2C_DIRECTION_RECEIVE)
  {
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &RegData, 1, I2C_LAST_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  rxcount++;
  if(rxcount == 1)
  {
    RegAdres = RxData[0];
    regIndex = findRegIndex(RegAdres);
  }
  if(rxcount < RxSIZE)
  {
    if (rxcount == RxSIZE-1)
    {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_LAST_FRAME);
    }
    else
    {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxcount, 1, I2C_NEXT_FRAME);
    }
  }

  if(rxcount == RxSIZE)
  {
    rxcount = 0;
    RegData = RxData[1];
    // Process data
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}
