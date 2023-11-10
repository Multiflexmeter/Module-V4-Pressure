#include "utils.h"
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;


/* Private functions */
int cmpfunc(const void* a, const void* b)
{
  return (*(int32_t*)a - *(int32_t*)b);
}

uint8_t getSlotID(void)
{
  uint8_t slotID;
  if(HAL_GPIO_ReadPin(SLOTID2_GPIO_Port, SLOTID2_Pin) == 0)
  {
    slotID = (HAL_GPIO_ReadPin(SLOTID1_GPIO_Port, SLOTID1_Pin) << 1) + HAL_GPIO_ReadPin(SLOTID0_GPIO_Port, SLOTID0_Pin);
  }
  else
  {
    slotID = (HAL_GPIO_ReadPin(SLOTID1_GPIO_Port, SLOTID1_Pin) << 1) + HAL_GPIO_ReadPin(SLOTID0_GPIO_Port, SLOTID0_Pin) + 3;
  }
  return slotID;
}

/* Public functions */

/**
 * @brief Sets the i2c slaveaddress based on the slot id
 */
void setSlaveAddress(void)
{
  uint8_t slotID = getSlotID();
  if(slotID <= 0)
    slotID = 1;

  uint8_t slaveAddress = ((0x11 + (slotID-1)) <<1) ;

  __HAL_I2C_DISABLE(&hi2c1);
  hi2c1.Instance->OAR1 &= ~I2C_OAR1_OA1EN;
  hi2c1.Instance->OAR1 = (I2C_OAR1_OA1EN | slaveAddress);
  __HAL_I2C_ENABLE(&hi2c1);
}

// Function for calculating median
float findMedian(float a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(float), cmpfunc);

  // check for even case
  if (n % 2 != 0)
      return (float)a[n / 2];

  return (float)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}

void enter_Sleep(void)
{
  /* Configure low-power mode */
  SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk );  // low-power mode = sleep mode

  /* Ensure Flash memory stays on */
  FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
  __WFI();  // enter low-power mode
}
