
#include "main.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;

enum state{ START1, BYTE1, START2, BYTE2, START3, BYTE3};
enum state dataState = START1;

int strobeTime = 0;
uint8_t bit_cnt = 0;
uint8_t byte1 = 0;
uint8_t byte2 = 0;
uint8_t byte3 = 0;
bool dataReady = false;
bool startup = true;

/**
 * @brief Delay function in microseconds
 *
 * @param us is the time to delay in microseconds
 */
void delay_us (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}


void HubaReceive(void)
{
  HAL_TIM_Base_Stop(&htim21);
  uint16_t time = TIM21->CNT;
  TIM21->CNT = 0;
  HAL_TIM_Base_Start(&htim21);

  // Reset if the strobe time is larger than 250us
  if(time > 250)
  {
    dataState = START1;
    byte1 = 0;
    byte2 = 0;
    byte3 = 0;
    bit_cnt = 0;
  }

  if(bit_cnt > 7)
  {
    bit_cnt = 0;
    dataState++;
    if(dataState > BYTE3)
    {
      dataState = START1;
      dataReady = true;
    }

    return;
  }

  switch(dataState)
  {
    case START1:
      if(startup)
      {
        startup = false;
      }
    case START2:
    case START3:
      TIM1->CNT = 0;
      HAL_TIM_Base_Start(&htim1);
      dataState++;
      break;

    case BYTE1:
      HAL_TIM_Base_Stop(&htim1);
      strobeTime = TIM1->CNT;
      delay_us((strobeTime/2) - 4);
      byte1 |= HAL_GPIO_ReadPin(One_wire2_GPIO_Port, One_wire2_Pin) << (7-bit_cnt);
      bit_cnt++;
      break;

    case BYTE2:
      HAL_TIM_Base_Stop(&htim1);
      strobeTime = TIM1->CNT;
      delay_us((strobeTime/2) - 4);
      byte2 |= HAL_GPIO_ReadPin(One_wire2_GPIO_Port, One_wire2_Pin) << (7-bit_cnt);
      bit_cnt++;
      break;

    case BYTE3:
      HAL_TIM_Base_Stop(&htim1);
      strobeTime = TIM1->CNT;
      delay_us((strobeTime/2) - 4);
      byte3 |= HAL_GPIO_ReadPin(One_wire2_GPIO_Port, One_wire2_Pin) << (7-bit_cnt);
      bit_cnt++;
      break;
  }
}
