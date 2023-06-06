/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void) {
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdHigh = (0x00) << 5;
  sFilterConfig.FilterIdLow = (0x100) << 5;
  sFilterConfig.FilterMaskIdHigh = (0x010) << 5;
  sFilterConfig.FilterMaskIdLow = (0x110) << 5;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
}

void sendCanTemp(uint8_t temp_fet, uint8_t temp_coil_1, uint8_t temp_coil_2) {
  CAN_TxHeaderTypeDef can_header;
  uint8_t can_data[8];
  uint32_t can_mailbox;

  can_header.StdId = 0x224;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.TransmitGlobalTime = DISABLE;
  can_data[0] = temp_fet;
  can_data[1] = temp_coil_1;
  can_data[2] = temp_coil_2;
  can_data[3] = 1;
  HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &can_mailbox);
}

void sendCanMouse(int16_t delta_x, int16_t delta_y, uint16_t quality) {
  CAN_TxHeaderTypeDef can_header;
  uint8_to_float_t tx;
  uint32_t can_mailbox;

  can_header.StdId = 0x240;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 4;
  can_header.TransmitGlobalTime = DISABLE;
  tx.mouse.delta_x = delta_x;
  tx.mouse.delta_y = delta_y;
  tx.mouse.quality = quality;
  HAL_CAN_AddTxMessage(&hcan, &can_header, tx.data, &can_mailbox);
}

void sendCanError(uint16_t type, uint32_t data) {
  CAN_TxHeaderTypeDef can_header;
  uint8_t can_data[8];
  uint32_t can_mailbox;

  can_header.StdId = 0x0;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.TransmitGlobalTime = DISABLE;
  can_data[0] = 0;
  can_data[1] = 0;
  can_data[2] = type;
  can_data[3] = type >> 8;
  HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &can_mailbox);
}

/* USER CODE END 1 */
