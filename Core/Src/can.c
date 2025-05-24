/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

uint8_t is_msp_initialized = 0;

CAN_FilterTypeDef CanFilter;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    printf("[CAN_INIT] Error initialization CAN module!\n");
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  printf("[CAN_INIT] CAN module initialized successfully!\n");

  CanFilter.FilterBank = 0;
  CanFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  CanFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  CanFilter.FilterIdHigh = 0x0000;
  CanFilter.FilterIdLow = 0x0000;
  CanFilter.FilterMaskIdHigh = 0x0000;
  CanFilter.FilterMaskIdLow = 0x0000;
  CanFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CanFilter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &CanFilter) != HAL_OK)
  {
    printf("[CAN_INIT] Error configuration CAN filter!\n");
    Error_Handler();
  }
  printf("[CAN_INIT] CAN filter configured successfully!\n");

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    printf("[CAN_INIT] Error activate CAN notification!\n");
	  Error_Handler();
  }
  printf("[CAN_INIT] CAN notification activated successfully!\n");

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    printf("[CAN_INIT] Error start CAN interface!\n");
    Error_Handler();
  }
  printf("[CAN_INIT] CAN interface started successfully! Low level initialized:%d\n", is_msp_initialized);



  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 15, 15);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
    is_msp_initialized = 1;
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void print_can_frame(const char* title, uint32_t id, uint8_t* rx_data, uint8_t dlc)
{
  printf("%s ID: %#05x, Data: ", title, id);
  for (int i = 0; i < dlc; i++)
  {
    const char* format = i == dlc - 1 ? "%02x\n" : "%02x:";
    printf(format, rx_data[i]);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef   RxHeader;
  uint8_t               RxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    printf("[CAN_RECEIVE] Error on receiving CAN frames!!!");
    Error_Handler();
  }

  print_can_frame("[CAN_RECEIVE] CAN message received!", RxHeader.StdId, RxData, RxHeader.DLC);
}

CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;

void can_send_message(uint32_t id, uint8_t* data, uint8_t dlc)
{
  if (data == NULL) {
    printf("[CAN_SEND] Cannot send CAN frame! Data ptr is NULL!!!");
    return;
  }

  dlc = dlc > 8 ? 8 : dlc;

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = dlc;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
  {
    printf("[CAN_SEND] Error sending CAN frame! TxMailbox: %d\n", TxMailbox);
    //Error_Handler();
    return;
  }

  print_can_frame("[CAN_SEND] CAN frame sent!", id, data, dlc);
}

/* USER CODE END 1 */
