#include "utils.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "SensorRegister.h"
#include "keller.h"
#include "modbus.h"

extern I2C_HandleTypeDef hi2c1;

#define SAMPLE_BUFFER_SIZE  10

/* Private functions */
int cmpfunc(const void* a, const void* b)
{
  return (*(int32_t*)a - *(int32_t*)b);
}

variant_t getVariant(void)
{
  return HAL_GPIO_ReadPin(VARIANT_DETECT_GPIO_Port, VARIANT_DETECT_Pin);
}

/**
 * @brief Determine the slot id based on the id inputs
 * @return The slot id
 */
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

/**
 * @brief Find the median of the given float array
 *
 * @param a The array of floats
 * @param n The lenght of the array
 * @return The median of the given array
 */
float findMedian(float a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(float), cmpfunc);

  // check for even case
  if (n % 2 != 0)
      return (float)a[n / 2];

  return (float)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}

/**
 * @brief Enable sensor 1
 */
void enableSensor1(void)
{
  HAL_GPIO_WritePin(SENSOR2_EN_GPIO_Port, SENSOR2_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Enable sensor 2
 */
void enableSensor2(void)
{
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR2_EN_GPIO_Port, SENSOR2_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Enable both sensors
 */
void enableSensors(void)
{
  HAL_GPIO_WritePin(SENSOR2_EN_GPIO_Port, SENSOR2_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Disable both sensors
 */
void disableSensors(void)
{
  HAL_GPIO_WritePin(SENSOR2_EN_GPIO_Port, SENSOR2_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief Enter sleep mode
 */
void enter_Sleep(void)
{
  /* Configure low-power mode */
  SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk );  // low-power mode = sleep mode

  /* Ensure Flash memory stays on */
  FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
  __WFI();  // enter low-power mode
}

/* State functions */
void assignAddressKeller(void)
{
  /* Enable the buck/boost */
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_SET);
  enableSensors();
  HAL_Delay(10);
  KellerSetBaudrate(0, BAUD_115200);
  disableSensors();

  /* Set address and baudrate of first sensor */
  enableSensor1();
  HAL_Delay(250);
  KellerInit(250);
  HAL_Delay(2);
  KellerNewAddress(250, 0x01);

  /* Set address and baudrate of second sensor */
  enableSensor2();
  HAL_Delay(250);
  KellerInit(250);
  HAL_Delay(2);
  KellerNewAddress(250, 0x02);

  /* Disable both sensors */
  disableSensors();
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_RESET);
}

void measureKellerSensor(void)
{
  uint16_t samples = readMeasSamples();
  float sensor1PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor1TempSamples[SAMPLE_BUFFER_SIZE];
  float sensor2PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor2TempSamples[SAMPLE_BUFFER_SIZE];
  SensorData sensorSample;

  /* Initialize both Keller sensors */
  bool sensor1Present = KellerInit(0x01);
  HAL_Delay(2);
  bool sensor2Present = KellerInit(0x02);

  /* Collect the samples specified in the MeasurementSamples register */
  for (uint8_t sample = 0; sample < samples; ++sample)
  {
    if(sensor1Present)
    {
      // Sample the first sensor
      memset(&sensorSample, 0, sizeof(SensorData));
      sensorSample = KellerReadTempAndPressure(0x01);
      sensor1PressureSamples[sample] = sensorSample.pressure;
      sensor1TempSamples[sample] = sensorSample.temperature;
      HAL_Delay(2);
    }

    if(sensor2Present)
    {
      // Sample the second sensor
      memset(&sensorSample, 0, sizeof(SensorData));
      sensorSample = KellerReadTempAndPressure(0x02);
      sensor2PressureSamples[sample] = sensorSample.pressure;
      sensor2TempSamples[sample] = sensorSample.temperature;
      HAL_Delay(2);
    }
  }

  /* Disable the buck/boost and store the median in the registers */
  disableSensors();
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_RESET);
  ModbusShutdown();
  storeMeasurement(findMedian(sensor1PressureSamples, samples), findMedian(sensor1TempSamples, samples), 0);
  storeMeasurement(findMedian(sensor2PressureSamples, samples), findMedian(sensor2TempSamples, samples), 1);
  setMeasurementStatus(MEASUREMENT_DONE);
  stopMeas();
  HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
}
