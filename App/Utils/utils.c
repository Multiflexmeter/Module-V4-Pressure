#include "utils.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "SensorRegister.h"
#include "keller.h"
#include "Huba.h"
#include "modbus.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;

HubaSensor hubaSensor1 = {
    .htim = &htim2,
    .bitIndex = 0,
    .hubaDone = false,
    .firstCapture = false
};

HubaSensor hubaSensor2 = {
    .htim = &htim21,
    .bitIndex = 0,
    .hubaDone = false,
    .firstCapture = false
};

#define SAMPLE_BUFFER_SIZE  10

/* Private functions */
int cmpfunc(const void* a, const void* b)
{
  return (*(int32_t*)a - *(int32_t*)b);
}

variant_t getVariant(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  variant_t variant = HAL_GPIO_ReadPin(VARIANT_DETECT_GPIO_Port, VARIANT_DETECT_Pin);
  __HAL_RCC_GPIOB_CLK_DISABLE();
  return variant;
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
 * @fn void controlSensor1(GPIO_PinState)
 * @brief function to control sensor 1
 *
 * @param state : GPIO_PIN_SET = enable, GPIO_PIN_RESET = disable
 */
void controlSensor1(GPIO_PinState state)
{
  HAL_GPIO_WritePin(SENSOR1_EN_GPIO_Port, SENSOR1_EN_Pin, !state );
}

/**
 * @fn void controlSensor2(GPIO_PinState)
 * @brief function to control sensor 2
 *
 * @param state : GPIO_PIN_SET = enable, GPIO_PIN_RESET = disable
 */
void controlSensor2(GPIO_PinState state)
{
  HAL_GPIO_WritePin(SENSOR2_EN_GPIO_Port, SENSOR2_EN_Pin, !state );
}

/**
 * @brief Enable sensor 1
 */
void enableSensor1(void)
{
  controlSensor1(GPIO_PIN_SET);
}

/**
 * @brief Enable sensor 2
 */
void enableSensor2(void)
{
  controlSensor2(GPIO_PIN_SET);
}

/**
 * @brief Disable both sensors
 */
void disableSensors(void)
{
  controlSensor1(GPIO_PIN_RESET);
  controlSensor2(GPIO_PIN_RESET);
}

/**
 * @brief Enter sleep mode
 */
void enter_Sleep(void)
{
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  HAL_ResumeTick();
}

/**
 * @fn void controlBuckConverter(GPIO_PinState)
 * @brief function to control buck converter
 *
 * @param state : GPIO_PIN_SET = enable, GPIO_PIN_RESET = disable
 */
void controlBuckConverter(GPIO_PinState state)
{
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, state);
}

/**
 * @fn void switchOnSensor_BothKeller(void)
 * @brief function to swtich on the buck converter and sensors 1 + 2.
 * After the buck a fixed delay of 5ms.
 * After the first sensor a fixed delay of 2ms
 * After the second sensor a fixed delay of 10ms
 */
void switchOnSensor_BothKeller(void)
{
  /* Enable the buck/boost */
  controlBuckConverter(GPIO_PIN_SET);

  HAL_Delay(5); //wait for stable supply for sensors

  /* switch on sensors one by one */
  enableSensor1(); //first enable sensor 1
  HAL_Delay(2);    //wait short while
  enableSensor2(); //second enable sensor 2

  HAL_Delay(10);

}

/**
 * @fn bool assignAddressKellerBroadcast(uint8_t)
 * @brief function first checks baudrate is 9600, force it to 115200, then set address
 *
 * @param address : new address of sensor
 * @return result of new address
 */
bool assignAddressKellerWithBroadcast(uint8_t address)
{
  //check sensor is at production baudrate, increase baudrate to 115200
  if( KellerCheckBaudrate(250, BAUD_9600) )
  {
    HAL_Delay(2);
    KellerSetBaudrate(250, BAUD_115200);
  }
  return KellerNewAddress(250, address);
}

/* State functions */
void assignAddressKeller(void)
{
  switchOnSensor_BothKeller();

  KellerSetBaudrate(0, BAUD_115200);
  disableSensors();

  /* Set address and baudrate of first sensor */
  enableSensor1();
  HAL_Delay(250);
  KellerInit(250);
  HAL_Delay(2);
  KellerNewAddress(250, 0x01);
  disableSensors(); //make sure sensor 1 is disabled.

  /* Set address and baudrate of second sensor */
  enableSensor2();
  HAL_Delay(250);
  KellerInit(250);
  HAL_Delay(2);
  KellerNewAddress(250, 0x02);

  /* Disable both sensors */
  disableSensors();
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter
}

/**
 * @fn void measureKellerSensor(void)
 * @brief function first polls sensor 1 and sensor 2.
 * Sensor must already be powered
 * Then it reads the sensor value for X times
 * After reading the sensors are disabled.
 * The buckbooster is disabled
 * The median value is calculated from the samples which are made
 *
 * @note: function does not repeat the initialize, because it's time consuming and costs extra power.
 * Repeat the initialize can be consider if problems with not found sensors remain.
 */
void measureKellerSensor(void)
{
  uint16_t samples = readMeasSamples();
  float sensor1PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor1TempSamples[SAMPLE_BUFFER_SIZE];
  float sensor2PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor2TempSamples[SAMPLE_BUFFER_SIZE];
  SensorData sensorSample;

  /* send dummy byte first, sometimes needed because Keller sensor does not response on first command */
  uint8_t data[1] = {0xFF};
  ModbusTransmitData(data, sizeof(data));
  HAL_Delay(2);

  /* Initialize both Keller sensors */
  bool sensor1Present = KellerInit(0x01);
  HAL_Delay(2);

  bool sensor2Present = KellerInit(0x02);
  HAL_Delay(2);

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
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter
  ModbusShutdown();
  storeMeasurement(findMedian(sensor1PressureSamples, samples), findMedian(sensor1TempSamples, samples), 0);
  storeMeasurement(findMedian(sensor2PressureSamples, samples), findMedian(sensor2TempSamples, samples), 1);
  setMeasurementStatus(MEASUREMENT_DONE);
  stopMeas();
}

void measureHubaSensor(void)
{
  uint8_t sample = 0;

  enableSensor1(); //enable sensor 1

  uint16_t samples = readMeasSamples();

  float sensor1PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor1TempSamples[SAMPLE_BUFFER_SIZE];
  float sensor2PressureSamples[SAMPLE_BUFFER_SIZE];
  float sensor2TempSamples[SAMPLE_BUFFER_SIZE];
  SensorData sensorSample;

  uint32_t timeout = HAL_GetTick() + 250;
  hubaStart(&hubaSensor1);
  while(sample < samples)
  {
    /* Sample first Huba sensor */
    if(hubaSensor1.hubaDone)
    {
      sensorSample = hubaBufferToData(&hubaSensor1);
      sensor1PressureSamples[sample] = sensorSample.pressure;
      sensor1TempSamples[sample] = sensorSample.temperature;
      hubaSensor1.hubaDone = false;
      sample++;
    }
    else if(HAL_GetTick() > timeout)
      break;
  }
  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
  disableSensors();

  HAL_Delay(5);
  enableSensor2();
  sample = 0;
  timeout = HAL_GetTick() + 250;
  hubaStart(&hubaSensor2);
  while(sample < samples)
  {
    /* Sample second Huba sensor */
    if(hubaSensor2.hubaDone)
    {
      sensorSample = hubaBufferToData(&hubaSensor2);
      sensor2PressureSamples[sample] = sensorSample.pressure;
      sensor2TempSamples[sample] = sensorSample.temperature;
      hubaSensor2.hubaDone = false;
      sample++;
    }
    else if(HAL_GetTick() > timeout)
      break;
  }
  HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter
  disableSensors();
  storeMeasurement(findMedian(sensor1PressureSamples, samples), findMedian(sensor1TempSamples, samples), 0);
  storeMeasurement(findMedian(sensor2PressureSamples, samples), findMedian(sensor2TempSamples, samples), 1);

  /* Finish measurement */
  setMeasurementStatus(MEASUREMENT_DONE);
  stopMeas();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim == hubaSensor1.htim)
  {
    hubaTimerCallback(&hubaSensor1);
  }
  else if(htim == hubaSensor2.htim)
  {
    hubaTimerCallback(&hubaSensor2);
  }
}
