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
#include "stm32f3xx_hal.h"

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
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOC
#define BOOST_SW_Pin GPIO_PIN_3
#define BOOST_SW_GPIO_Port GPIOA
#define LED_CURRENT_Pin GPIO_PIN_4
#define LED_CURRENT_GPIO_Port GPIOA
#define KICK_1_Pin GPIO_PIN_6
#define KICK_1_GPIO_Port GPIOA
#define KICK_2_Pin GPIO_PIN_7
#define KICK_2_GPIO_Port GPIOA
#define BOOST_V_Pin GPIO_PIN_0
#define BOOST_V_GPIO_Port GPIOB
#define BATT_CS_Pin GPIO_PIN_1
#define BATT_CS_GPIO_Port GPIOB
#define POWER_SW_EN_Pin GPIO_PIN_2
#define POWER_SW_EN_GPIO_Port GPIOB
#define TEMP_FET_Pin GPIO_PIN_12
#define TEMP_FET_GPIO_Port GPIOB
#define TEMP_COIL_1_Pin GPIO_PIN_13
#define TEMP_COIL_1_GPIO_Port GPIOB
#define TEMP_COIL_2_Pin GPIO_PIN_14
#define TEMP_COIL_2_GPIO_Port GPIOB
#define MOUSE_NSS_Pin GPIO_PIN_15
#define MOUSE_NSS_GPIO_Port GPIOA
#define MOUSE_RST_Pin GPIO_PIN_6
#define MOUSE_RST_GPIO_Port GPIOB
#define GD_16M_PWM_Pin GPIO_PIN_7
#define GD_16M_PWM_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_8
#define SW_1_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_9
#define SW_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
