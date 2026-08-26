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
#include <stdbool.h>
#include <string.h>

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
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef * canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN) {
    /* USER CODE BEGIN CAN_MspInit 0 */

    /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    /* USER CODE BEGIN CAN_MspInit 1 */

    /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef * canHandle)
{
  if (canHandle->Instance == CAN) {
    /* USER CODE BEGIN CAN_MspDeInit 0 */

    /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
    /* USER CODE BEGIN CAN_MspDeInit 1 */

    /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdHigh = (0x000) << 5;      // エラー
  sFilterConfig.FilterIdLow = (0x001) << 5;       // リセット
  sFilterConfig.FilterMaskIdHigh = (0x010) << 5;  // パラメーター設定
  sFilterConfig.FilterMaskIdLow = (0x110) << 5;   // キッカー制御
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  sFilterConfig.FilterBank = 2;
  sFilterConfig.FilterIdHigh = (0x611) << 5;
  sFilterConfig.FilterIdLow = (0x611) << 5;
  sFilterConfig.FilterMaskIdHigh = (0x611) << 5;
  sFilterConfig.FilterMaskIdLow = (0x611) << 5;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* OTA entry ID 0x600を通常制御IDとは別bankで常時受信する。 */
  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterIdHigh = (0x600) << 5;
  sFilterConfig.FilterIdLow = (0x600) << 5;
  sFilterConfig.FilterMaskIdHigh = (0x600) << 5;
  sFilterConfig.FilterMaskIdLow = (0x600) << 5;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    Error_Handler();
  }
}

#define CAN_TX_FIFO_CAPACITY 32U
#define CAN_TX_FIFO_STORAGE (CAN_TX_FIFO_CAPACITY + 1U)

typedef struct
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
} can_tx_frame_t;

static can_tx_frame_t can_tx_fifo[CAN_TX_FIFO_STORAGE];
static volatile uint8_t can_tx_head;
static volatile uint8_t can_tx_tail;
static volatile uint32_t can_tx_drop_count;

static uint8_t can_tx_next(uint8_t index)
{
  index++;
  return index < CAN_TX_FIFO_STORAGE ? index : 0U;
}

/* Called with interrupts disabled. Fill every free hardware mailbox in FIFO order. */
static void can_tx_drain_locked(void)
{
  while (can_tx_tail != can_tx_head && HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0U) {
    uint32_t mailbox;
    const can_tx_frame_t * frame = &can_tx_fifo[can_tx_tail];
    if (HAL_CAN_AddTxMessage(&hcan, &frame->header, frame->data, &mailbox) != HAL_OK) break;
    can_tx_tail = can_tx_next(can_tx_tail);
  }
}

static bool can_tx_enqueue(uint32_t id, uint32_t dlc, const uint8_t data[8])
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint8_t next = can_tx_next(can_tx_head);
  if (next == can_tx_tail) {
    can_tx_drop_count++;
    __set_PRIMASK(primask);
    return false;
  }

  can_tx_frame_t * frame = &can_tx_fifo[can_tx_head];
  memset(frame, 0, sizeof(*frame));
  frame->header.StdId = id;
  frame->header.RTR = CAN_RTR_DATA;
  frame->header.IDE = CAN_ID_STD;
  frame->header.DLC = dlc;
  frame->header.TransmitGlobalTime = DISABLE;
  memcpy(frame->data, data, 8U);
  __DMB();
  can_tx_head = next;
  can_tx_drain_locked();
  __set_PRIMASK(primask);
  return true;
}

static void can_tx_mailbox_complete(CAN_HandleTypeDef * handle)
{
  if (handle != &hcan) return;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  can_tx_drain_locked();
  __set_PRIMASK(primask);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef * handle) { can_tx_mailbox_complete(handle); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef * handle) { can_tx_mailbox_complete(handle); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef * handle) { can_tx_mailbox_complete(handle); }

void sendCanTemp(uint8_t temp_fet, uint8_t temp_coil_1, uint8_t temp_coil_2)
{
  const uint8_t data[8] = {temp_fet, temp_coil_1, temp_coil_2, 1U, 0U, 0U, 0U, 0U};
  (void)can_tx_enqueue(0x224U, 8U, data);
}

void sendCanMouse(int16_t delta_x, int16_t delta_y, uint16_t quality)
{
  can_msg_buf_t tx = {0};
  tx.mouse.delta_x = delta_x;
  tx.mouse.delta_y = delta_y;
  tx.mouse.quality = quality;
  (void)can_tx_enqueue(0x241U, 6U, tx.data);
}

void sendCanError(uint16_t info, float value)
{
  can_msg_buf_t tx = {0};
  tx.error.node_id = 100;
  tx.error.info = info;
  tx.error.value = value;
  (void)can_tx_enqueue(0x000U, 8U, tx.data);
}

void sendFloat(uint32_t can_id, float data)
{
  can_msg_buf_t tx = {0};
  tx.voltage.value = data;
  (void)can_tx_enqueue(can_id, 4U, tx.data);
}

void sendFirmwareVersion(uint32_t build_id, uint32_t image_crc32c)
{
  uint8_t data[8];
  memcpy(&data[0], &build_id, sizeof(build_id));
  memcpy(&data[4], &image_crc32c, sizeof(image_crc32c));
  (void)can_tx_enqueue(0x6C4U, 8U, data);
}

void sendCanPowerStatus(uint8_t flags)
{
  uint8_t data[8] = {flags, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  (void)can_tx_enqueue(0x244U, 8U, data);
}

void sendCanBatteryVoltage(float voltage) { sendFloat(0x215, voltage); }
void sendCanKickerVoltage(float voltage) { sendFloat(0x216, voltage); }
void sendCanBatteryCurrent(float current) { sendFloat(0x234, current); }

/* USER CODE END 1 */
