/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_SW2_Pin GPIO_PIN_0
#define DEBUG_SW2_GPIO_Port GPIOC
#define DEBUG_SW1_Pin GPIO_PIN_13
#define DEBUG_SW1_GPIO_Port GPIOC
#define SLOTID1_Pin GPIO_PIN_0
#define SLOTID1_GPIO_Port GPIOA
#define USART_TX_EN_Pin GPIO_PIN_1
#define USART_TX_EN_GPIO_Port GPIOA
#define SLOTID2_Pin GPIO_PIN_4
#define SLOTID2_GPIO_Port GPIOA
#define SLOTID3_Pin GPIO_PIN_5
#define SLOTID3_GPIO_Port GPIOA
#define SLOT_GPIO0_Pin GPIO_PIN_0
#define SLOT_GPIO0_GPIO_Port GPIOB
#define SLOT_GPIO2_Pin GPIO_PIN_1
#define SLOT_GPIO2_GPIO_Port GPIOB
#define SLOT_GPIO2B2_Pin GPIO_PIN_2
#define SLOT_GPIO2B2_GPIO_Port GPIOB
#define DEBUG_LED2_Pin GPIO_PIN_14
#define DEBUG_LED2_GPIO_Port GPIOB
#define DEBUG_LED1_Pin GPIO_PIN_15
#define DEBUG_LED1_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_8
#define INT_GPIO_Port GPIOA
#define USART_RX_EN_Pin GPIO_PIN_12
#define USART_RX_EN_GPIO_Port GPIOA
#define BUCK_EN_Pin GPIO_PIN_3
#define BUCK_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
