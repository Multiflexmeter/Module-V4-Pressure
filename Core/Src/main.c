/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*! \mainpage Main Page
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 * \section general_sec General information
 *
 * General information
 *
 * \section install_sec Installation
 *
 * Information for installation.
 *
 * \subsection step1 Step 1: Opening the box
 *
 * etc...
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "SensorRegister.h"
#include "I2C_Slave.h"
#include "keller.h"
#include "modbus.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  POLL_SENSOR,
  WRITE_REGISTER,
  SLEEP,
} state_machine_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_BUFFER_SIZE  10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint16_t supplyVSENSORSLOT;
extern uint16_t supply3V3;
extern bool writeFlag;

int32_t sensor1Samples[SAMPLE_BUFFER_SIZE];
int32_t sensor2Samples[SAMPLE_BUFFER_SIZE];

state_machine_t currentState = SLEEP;
uint16_t samples;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to compare two uint16_t for qsort
int cmpfunc(const void* a, const void* b)
{
  return (*(int32_t*)a - *(int32_t*)b);
}

// Function for calculating median
int32_t findMedian(int32_t a[], uint8_t n)
{
  // First we sort the array
  qsort(a, n, sizeof(int32_t), cmpfunc);

  // check for even case
  if (n % 2 != 0)
      return (int32_t)a[n / 2];

  return (int32_t)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
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

void setSlaveAddress(uint8_t slotID)
{
  if(slotID <= 0)
    slotID = 1;

  uint8_t slaveAddress = (0x11<<1) + (slotID-1);

  __HAL_I2C_DISABLE(&hi2c1);
  hi2c1.Instance->OAR1 &= ~I2C_OAR1_OA1EN;
  hi2c1.Instance->OAR1 = (I2C_OAR1_OA1EN | slaveAddress);
  __HAL_I2C_ENABLE(&hi2c1);
}

void enableSensor1(void)
{
  HAL_GPIO_WritePin(SENSOR1_ENABLE_GPIO_Port, SENSOR1_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR0_ENABLE_GPIO_Port, SENSOR0_ENABLE_Pin, GPIO_PIN_RESET);
}

void disableSensors(void)
{
  HAL_GPIO_WritePin(SENSOR0_ENABLE_GPIO_Port, SENSOR0_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR1_ENABLE_GPIO_Port, SENSOR1_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_RESET);
}

void enableSensor2(void)
{
  HAL_GPIO_WritePin(SENSOR0_ENABLE_GPIO_Port, SENSOR0_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR1_ENABLE_GPIO_Port, SENSOR1_ENABLE_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ModbusInit(&huart2);
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_I2C_EnableListen_IT(&hi2c1);
  setSlaveAddress(getSlotID());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ADC_Start(&hadc);
    switch (currentState)
    {
      case POLL_SENSOR:
        /* Initialize both Keller sensors */
        KellerInit(0x01);
        KellerInit(0x02);

        /* Collect the samples specified in the MeasurementSamples register */
        for (uint8_t sample = 0; sample < samples; ++sample)
        {
          //todo sample pressure and temp of both sensors
          sensor1Samples[sample] = KellerReadTemperature(0x02);
          HAL_Delay(1);
          sensor2Samples[sample] = KellerReadPressure(0x02);
          HAL_Delay(1);
        }

        /* Disable the buck/boost and store the median in the registers */
        HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_RESET);
        ModbusShutdown();
        storeMeasurement(findMedian(sensor2Samples, samples), findMedian(sensor1Samples, samples), 0);
        setMeasurementStatus(MEASUREMENT_DONE);
        stopMeas();
        currentState = SLEEP;
        break;

      case WRITE_REGISTER:
        if(regWriteData[0] == REG_SENSOR_SELECTED)
        {
          writeRegister(regWriteData, regSize+3);
          storeSelectedSensor(regWriteData[1]);
        }
        else
          writeRegister(regWriteData, regSize+3);

        writeFlag = false;
        currentState = SLEEP;
        break;

      case SLEEP:
        if(writeFlag)
          currentState = WRITE_REGISTER;

        if(readMeasStart())
        {
          currentState = POLL_SENSOR;
          setMeasurementStatus(MEASUREMENT_ACTIVE);
          samples = readMeasSamples();
          HAL_GPIO_WritePin(BUCK_EN_GPIO_Port, BUCK_EN_Pin, GPIO_PIN_SET);
          ModbusShutdown();
          HAL_Delay(250);
        }
        else
        {
          HAL_PWR_DisableSleepOnExit();
          HAL_SuspendTick();
          HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
          HAL_ResumeTick();
          HAL_Delay(30);
        }

        break;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR0_ENABLE_GPIO_Port, SENSOR0_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USART_TX_EN_Pin|INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SLOT_GPIO0_Pin|SLOT_GPIO1_Pin|SLOT_GPIO2_Pin|DEBUG_LED2_Pin
                          |DEBUG_LED1_Pin|BUCK_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR1_ENABLE_GPIO_Port, SENSOR1_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART_RX_EN_GPIO_Port, USART_RX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DEBUG_SW2_Pin */
  GPIO_InitStruct.Pin = DEBUG_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEBUG_SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR0_ENABLE_Pin */
  GPIO_InitStruct.Pin = SENSOR0_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR0_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SLOTID0_Pin SLOTID1_Pin SLOTID2_Pin */
  GPIO_InitStruct.Pin = SLOTID0_Pin|SLOTID1_Pin|SLOTID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_EN_Pin INT_Pin USART_RX_EN_Pin */
  GPIO_InitStruct.Pin = USART_TX_EN_Pin|INT_Pin|USART_RX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SLOT_GPIO0_Pin SLOT_GPIO1_Pin SLOT_GPIO2_Pin SENSOR1_ENABLE_Pin
                           DEBUG_LED2_Pin DEBUG_LED1_Pin BUCK_EN_Pin */
  GPIO_InitStruct.Pin = SLOT_GPIO0_Pin|SLOT_GPIO1_Pin|SLOT_GPIO2_Pin|SENSOR1_ENABLE_Pin
                          |DEBUG_LED2_Pin|DEBUG_LED1_Pin|BUCK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB13 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
