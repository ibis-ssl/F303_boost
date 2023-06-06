/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
typedef union {
  uint8_t data[8];

  struct {
    uint8_t idx;
    float value;
  } power;
  struct {
    uint16_t node_id;
    uint16_t type;
    uint32_t etc;
  } error_notify;

  struct {
    uint8_t cmd;
    uint8_t enable;
  } power_en;
  struct {
    uint8_t idx;
    float value;
  } set_protect_param;
  struct {
    int16_t delta_x;
    int16_t delta_y;
    uint16_t quality;
  } mouse;
} uint8_to_float_t;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */


void CAN_Filter_Init(void);

void sendCanTemp(uint8_t temp_fet, uint8_t temp_coil_1, uint8_t temp_coil_2);
void sendCanMouse(int16_t delta_x, int16_t delta_y, uint16_t quality);
void sendCanError(uint16_t type, uint32_t data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

