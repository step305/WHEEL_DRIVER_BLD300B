/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define WHEEL1_SPEED_Pin GPIO_PIN_0
#define WHEEL1_SPEED_GPIO_Port GPIOA
#define WHEEL2_SPEED_Pin GPIO_PIN_1
#define WHEEL2_SPEED_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define WHEEL1_TORQ_Pin GPIO_PIN_4
#define WHEEL1_TORQ_GPIO_Port GPIOA
#define WHEEL2_TORQ_Pin GPIO_PIN_5
#define WHEEL2_TORQ_GPIO_Port GPIOA
#define RIGHT_TURN_LIGHT_Pin GPIO_PIN_0
#define RIGHT_TURN_LIGHT_GPIO_Port GPIOB
#define STOP_SIGNAL_Pin GPIO_PIN_9
#define STOP_SIGNAL_GPIO_Port GPIOA
#define REVERSE_LIGHTS_Pin GPIO_PIN_10
#define REVERSE_LIGHTS_GPIO_Port GPIOA
#define WHEEL1_BRK_Pin GPIO_PIN_11
#define WHEEL1_BRK_GPIO_Port GPIOA
#define LEFT_TURN_LIGHT_Pin GPIO_PIN_12
#define LEFT_TURN_LIGHT_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define WHEEL2_FR_Pin GPIO_PIN_15
#define WHEEL2_FR_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define WHEEL1_EN_Pin GPIO_PIN_4
#define WHEEL1_EN_GPIO_Port GPIOB
#define WHEEL1_FR_Pin GPIO_PIN_5
#define WHEEL1_FR_GPIO_Port GPIOB
#define WHEEL2_BRK_Pin GPIO_PIN_6
#define WHEEL2_BRK_GPIO_Port GPIOB
#define WHEEL2_EN_Pin GPIO_PIN_7
#define WHEEL2_EN_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
