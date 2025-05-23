/**
  ******************************************************************************
  * @file           Utils.c
  * @brief          Utils functions, general helper functions
  * @author         D.Kerstens
  * @date           Nov 9, 2023
  ******************************************************************************
  */

#include "utils.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "SensorRegister.h"
#include "keller.h"
#include "Huba.h"
#include "modbus.h"
#include "adc.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;
extern ADC_HandleTypeDef hadc;

extern uint16_t supplyVSENSOR;

HubaSensor hubaSensor[DEF_SENSOR_AMOUNT] = //
{ //
    { //sensor 1
        .htim = &htim2, //
        .bitIndex = 0, //
        .hubaDone = false, //
        .firstCapture = false, //
    }//end sensor 1
    ,//
    { //sensor 2
        .htim = &htim21, //
        .bitIndex = 0, //
        .hubaDone = false, //
        .firstCapture = false, //
    }//end sensor 2
};

#define SAMPLE_BUFFER_SIZE  10
#define SAMPLE_MAX_BUFFER_SIZE  100

float sensorPressureSamplesKeller[DEF_SENSOR_AMOUNT][SAMPLE_MAX_BUFFER_SIZE];
float sensorTempSamplesKeller[DEF_SENSOR_AMOUNT][SAMPLE_MAX_BUFFER_SIZE];

uint16_t sensorPressureSamplesHuba[DEF_SENSOR_AMOUNT][SAMPLE_MAX_BUFFER_SIZE];
uint8_t sensorTempSamplesHuba[DEF_SENSOR_AMOUNT][SAMPLE_MAX_BUFFER_SIZE];

/* Private functions */
/**
 * @fn int cmpfunc(const void*, const void*)
 * @brief compare help function for floats32 used with qsort()
 *
 * @param a first value in float
 * @param b second value in float
 * @return result : negative a < b, 0 a = b, positive a > b
 */
int cmpfunc(const void* a, const void* b)
{
  return (*(int32_t*)a - *(int32_t*)b);
}

/**
 * @fn int cmpfunc_uint8(const void*, const void*)
 * @brief compare help function for uint8 used with qsort()
 *
 * @param a first value uint8
 * @param b second value uint8
 * @return result : negative a < b, 0 a = b, positive a > b
 */
int cmpfunc_uint8(const void* a, const void* b)
{
  return (*(uint8_t*)a - *(uint8_t*)b);
}

/**
 * @fn int cmpfunc_uint16(const void*, const void*)
 * @brief compare help function for uint16 used with qsort()
 *
 * @param a first value uint16
 * @param b second value uint16
 * @return result : negative a < b, 0 a = b, positive a > b
 */
int cmpfunc_uint16(const void* a, const void* b)
{
  return (*(uint16_t*)a - *(uint16_t*)b);
}

/**
 * @fn variant_t getVariant(void)
 * @brief function returns the board variant
 *
 * @return \ref variant_t RS485 or ONEWIRE
 */
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
float findMedian_float(float a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(float), cmpfunc);

  // check for even case
  if (n % 2 != 0)
      return (float)a[n / 2];

  return (float)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}

/**
 * @brief Find the median of the given uint8 array
 *
 * @param a The array of uint8
 * @param n The lenght of the array
 * @return The median of the given array
 */
uint8_t findMedian_uint8(uint8_t a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(uint8_t), cmpfunc_uint8);

  // check for even case
  if (n % 2 != 0)
      return (uint8_t)a[n / 2];

  return (uint8_t)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}

/**
 * @brief Find the median of the given uint16 array
 *
 * @param a The array of uint16
 * @param n The lenght of the array
 * @return The median of the given array
 */
uint16_t findMedian_uint16(uint16_t a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(uint16_t), cmpfunc_uint16);

  // check for even case
  if (n % 2 != 0)
      return (uint16_t)a[n / 2];

  return (uint16_t)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
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
 * @brief Enables the current limiting resistor
 * Only available from PCB revision 1.3
 */
void enableCurrentLimit(void)
{
  HAL_GPIO_WritePin(CURRENT_SW_GPIO_Port, CURRENT_SW_Pin, 1);
}

/**
 * @brief Disables the current limiting resistor
 * Only available from PCB revision 1.3
 */
void disableCurrentLimit(void)
{
  HAL_GPIO_WritePin(CURRENT_SW_GPIO_Port, CURRENT_SW_Pin, 0);
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

  HAL_Delay(25); //wait for stabilized supply. Needs to be 25ms minimum for guaranteeing stable output voltage.

  /* switch on sensors one by one */
  disableCurrentLimit();
  enableSensor1(); //first enable sensor 1
  HAL_Delay(2);    //wait short while
  enableSensor2(); //second enable sensor 2

  HAL_Delay(10);

}

/**
 * @fn bool assignAddressKellerWithBroadcast(uint8_t)
 * @brief function first checks baudrate is 9600, force it to 115200, then set address
 *
 * @param address : new address of sensor
 * @return result of new address. true = succeed, false = failure
 */
bool assignAddressKellerWithBroadcast(uint8_t address)
{
  int retry = 3;
  //check sensor is at production baudrate, increase baudrate to 115200
  if( KellerCheckBaudrate(250, BAUD_9600) )
  {
    HAL_Delay(2);
    KellerSetBaudrate(250, BAUD_115200);
    HAL_Delay(10);
  }

  HAL_Delay(2);
  // check if sensor works at 115200,
  // retry must be used, because after chaning baudrate the sensor does not response on first message
  while( KellerCheckBaudrate(250, BAUD_115200) == false && retry )
  {
    retry--;
    HAL_Delay(2);
  }

  //set new address, verify result is equal to address, then succeed.
  return (KellerNewAddress(250, address) == address);
  }

/**
 * @fn bool assignAddressKeller(void)
 * @brief function to assign address for keller sensors
 *
 * @return true = succeed, false = failure
 */
bool assignAddressKellerBothSensors(void)
{
  bool resultSensor1 = 0;
  bool resultSensor2 = 0;
  controlBuckConverter(GPIO_PIN_SET);

  HAL_Delay(5); //wait for stable supply for sensors

  /* Set address and baudrate of first sensor */
  enableSensor1();
  HAL_Delay(250);
  resultSensor1 = assignAddressKellerWithBroadcast(0x01);

  disableSensors(); //make sure sensor 1 is disabled.

  /* Set address and baudrate of second sensor */
  enableSensor2();
  HAL_Delay(250);
  resultSensor2 = assignAddressKellerWithBroadcast(0x02);

  /* Disable both sensors */
  disableSensors();
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter

  return (resultSensor1 == true && resultSensor2 == true);
}

/**
 * @fn bool assignAddressKeller(uint8_t)
 * @brief function to assign address for keller sensors
 *
 * @param sensor : number of sensor default start at 0. Accept 0 - 1
 * @return true = succeed, false = failure
 */
bool assignAddressKeller(uint8_t sensor)
{
  bool resultSensor = false;

  /* check sensor input is valid */
  if( sensor >= DEF_SENSOR_AMOUNT )
  {
    return resultSensor;
  }

  /* enable supply */
  controlBuckConverter(GPIO_PIN_SET);

  HAL_Delay(5); //wait for stable supply for sensors

  if( sensor == 0 )
  {
    /* Set address and baudrate of first sensor */
    enableSensor1();
  }
  else if( sensor == 1)
  {
    /* Set address and baudrate of second sensor */
    enableSensor2();
  }
  else //not valid
  {
    //nothing cannot happen

    static_assert( DEF_SENSOR_AMOUNT == 2, "function not suitable for this amount of sensors");
  }

  HAL_Delay(250); //wait
  resultSensor = assignAddressKellerWithBroadcast(sensor + 1); //assign address 0=0x01, 1=0x02

  /* Disable both sensors */
  disableSensors();
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter

  return (resultSensor == true );
}

/**
 * @fn uint16_t getNumberOfSamples(void)
 * @brief helper function to minimize and maximize the number of samples readed from register.
 *
 * @return samples from 1 - 100
 */
uint16_t getNumberOfSamples(void)
{
  uint16_t samples = readMeasSamples();

  if ( samples == 0 ) //guard no zero
  {
    return 1;
  }

  if( samples > SAMPLE_MAX_BUFFER_SIZE) //guard maximum
  {
    return SAMPLE_MAX_BUFFER_SIZE;
  }

  return samples;
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
  SensorData sensorSample;
  bool sensorPresent[DEF_SENSOR_AMOUNT];
  uint16_t samples = getNumberOfSamples();

  /* send dummy byte first, sometimes needed because Keller sensor does not response on first command */
  uint8_t data[1] = {0xFF};
  ModbusTransmitData(data, sizeof(data));
  HAL_Delay(2);

  /* Initialize both Keller sensors */
  for( int sensorNr=0; sensorNr < DEF_SENSOR_AMOUNT; sensorNr++)
  {
    sensorPresent[sensorNr] = KellerInit( sensorNr+0x01 ); //Initialize the sensor 1 and sensor 2
    HAL_Delay(2);
  }

  /* Collect the samples specified in the MeasurementSamples register */
  for (uint8_t sample = 0; sample < samples; ++sample)
  {
    for (int sensorNr = 0; sensorNr < DEF_SENSOR_AMOUNT; sensorNr++)
    {
      if (sensorPresent[sensorNr]) //check sensor is present
      {
        memset(&sensorSample, 0, sizeof(SensorData)); //clear memory
        sensorSample = KellerReadTempAndPressure(sensorNr+0x01);
        sensorPressureSamplesKeller[sensorNr][sample] = sensorSample.pressureData;
        sensorTempSamplesKeller[sensorNr][sample] = sensorSample.temperatureData;
        HAL_Delay(2);
      }
    }
  }

  // Check the sensor power supply voltage
  supplyVSENSOR = ADC_Vsensor_Measure(&hadc);
  if(supplyVSENSOR <= 2700 || supplyVSENSOR >= 3900)
    setErrorCode(VSENSOR_ERROR);

  /* Disable the buck/boost and store the median in the registers */
  disableSensors();
  enableCurrentLimit();
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter
  ModbusShutdown();

  for( int sensorNr=0; sensorNr < DEF_SENSOR_AMOUNT; sensorNr++)
  {
    clearMeasurement( sensorNr );

    if( sensorPresent[sensorNr] )
    {
      storeMeasurementKeller(findMedian_float(sensorPressureSamplesKeller[sensorNr], samples), findMedian_float(sensorTempSamplesKeller[sensorNr], samples), sensorNr);
    }
  }

  setMeasurementStatus(MEASUREMENT_DONE);
  stopMeas();
}

/**
 * @fn void measureHubaSensor(void)
 * @brief function reads available onewire sensors in a loop.
 * Number of samples is read, sensor supply is enabled
 * Sensor must already be powered
 * Then it reads the sensor value for X times
 * After reading the sensors are disabled.
 * The buckbooster is disabled finally
 * The median value is calculated from the samples which are made and stored
 */
void measureHubaSensor(void)
{
  uint8_t sample = 0;
  SensorDataHuba sensorSample;
  uint16_t samples = getNumberOfSamples();
  bool sensorFound[DEF_SENSOR_AMOUNT] = {0};

  for( int sensorNr = 0; sensorNr < DEF_SENSOR_AMOUNT; sensorNr++)
  {
    sample = 0;
    uint32_t timeout = HAL_GetTick() + 10 + (samples * 22) / 10;
    switch( sensorNr )
    {
      case 0:
        enableSensor1(); //enable sensor 1
        break;
      case 1:
        enableSensor2(); //enable sensor 2
        break;
      default:
        //nothing, wrong value
        break;
    }

    hubaStart(&hubaSensor[sensorNr]);
    while(sample < samples)
    {
      /* Sample first Huba sensor */
      if(hubaSensor[sensorNr].hubaDone)
      {
        sensorFound[sensorNr] = true;
        sensorSample = hubaBufferToData(&hubaSensor[sensorNr]);
        sensorPressureSamplesHuba[sensorNr][sample] = sensorSample.pressureData;
        sensorTempSamplesHuba[sensorNr][sample] = sensorSample.temperatureData;
        hubaSensor[sensorNr].hubaDone = false;
        sample++;
      }
      else if(HAL_GetTick() > timeout)
        break;
    }

    HAL_TIM_IC_Stop_IT(hubaSensor[sensorNr].htim, TIM_CHANNEL_1);

    // Check the sensor power supply voltage
    supplyVSENSOR = ADC_Vsensor_Measure(&hadc);
    if(supplyVSENSOR <= 4500 || supplyVSENSOR >= 5500)
      setErrorCode(VSENSOR_ERROR);

    disableSensors();

    // set delay between, not at last cycle.
    if( sensorNr + 1 < DEF_SENSOR_AMOUNT )
    {
      HAL_Delay(5);
    }
  }

  enableCurrentLimit();
  controlBuckConverter(GPIO_PIN_RESET); //disable buck converter

  for( int sensorNr=0; sensorNr<DEF_SENSOR_AMOUNT; sensorNr++)
  {
    clearMeasurement(sensorNr);
    if (sensorFound[sensorNr])
    {
      storeMeasurementHuba(findMedian_uint16(sensorPressureSamplesHuba[sensorNr], samples), findMedian_uint8(sensorTempSamplesHuba[sensorNr], samples), sensorNr);
    }
  }

  /* Finish measurement */
  setMeasurementStatus(MEASUREMENT_DONE);
  stopMeas();
}

/**
 * @fn void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*)
 * @brief  Input Capture callback in non-blocking mode
 *
 * @param  htim TIM IC handle
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim == hubaSensor[0].htim)
  {
    hubaTimerCallback(&hubaSensor[0]);
  }
  else if(htim == hubaSensor[1].htim)
  {
    hubaTimerCallback(&hubaSensor[1]);
  }
}
