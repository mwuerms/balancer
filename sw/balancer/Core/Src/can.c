/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define MAX_DATA_SIZE (8)
#define NB_MOTORS (2)
#define MOTOR_CAN_ADDR_0 (0x00B) // 11 = 0x00B 0b1011
#define MOTOR_CAN_ADDR_1 (0x00C) // 12 = 0x00C 0x1100
#define MOTOR_CAN_ADDR_FILT_MASK (0x008 )// 0b1000
// bitrate 500000 bps

typedef struct {
	CAN_TxHeaderTypeDef hdr;
	uint32_t mailbox;
	uint8_t data[MAX_DATA_SIZE];
} can_def_t;
static can_def_t motors[NB_MOTORS];

static CAN_RxHeaderTypeDef in_header;
static uint8_t in_data[8];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_LOOPBACK;//CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  // filter configuration
  CAN_FilterTypeDef can_filter;

  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.FilterBank = 10;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterIdHigh = 0;//MOTOR_CAN_ADDR_FILT_MASK<<5;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0;//MOTOR_CAN_ADDR_FILT_MASK<<5;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &can_filter);

  /* USER CODE END CAN_Init 2 */

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

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

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

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static volatile uint16_t count = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &in_header, in_data);
	count++;
	return;
}

uint16_t can_addr = 0;
void can_start(void) {
	motors[MOTOR0].hdr.ExtId = 0;
	motors[MOTOR0].hdr.IDE = CAN_ID_STD;
	motors[MOTOR0].hdr.StdId = MOTOR_CAN_ADDR_0;
	motors[MOTOR0].hdr.TransmitGlobalTime = DISABLE;
	motors[MOTOR1].hdr.ExtId = 0;
	motors[MOTOR1].hdr.IDE = CAN_ID_STD;
	motors[MOTOR1].hdr.TransmitGlobalTime = DISABLE;
	motors[MOTOR1].hdr.StdId = MOTOR_CAN_ADDR_1;

	can_addr = 0;
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*
void can_motor_send(can_def_t motor) {
	uint8_t n;
	if(len > 8) {
		len = 8;
	}

	for(n = 0; n < len; n++) {
		out_data[n] = send_data[n];
	}
	out_header.DLC = len;
	out_header.ExtId = 0;
	out_header.IDE = CAN_ID_STD;
	out_header.RTR = CAN_RTR_DATA;
	out_header.StdId = addr;
	out_header.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(&hcan, &out_header, out_data, &out_mailbox);
}*/

#define MOTOR0 (0)
#define MOTOR1 (1)
void can_hello_motor(uint8_t motor) {
	// only MOTOR0, MOTOR1
	if(motor >= MOTOR1) {
		motor = MOTOR1;
	}
	if(can_addr >= 64) {
		return;
	}

	HAL_CAN_AbortTxRequest(&hcan, &motors[motor].mailbox);
	//motors[MOTOR0].hdr.StdId = (MOTOR_CAN_ADDR_0 << 5) | 5;
	motors[MOTOR0].hdr.StdId = (can_addr << 5) | 0;
	//motors[motor].hdr.RTR = CAN_RTR_DATA; // send data to remote
	motors[motor].hdr.RTR = CAN_RTR_REMOTE; // request data from remote
	motors[motor].data[0] = 5; //serial_number, 64 bit (8 bytes)
	motors[motor].data[1] = 0;
	motors[motor].data[2] = 0;
	motors[motor].data[3] = 0;
	motors[motor].data[4] = 0;
	motors[motor].data[5] = 0;
	motors[motor].data[6] = 0;
	motors[motor].data[7] = 0;
	motors[motor].hdr.DLC = 0;//len;
	HAL_CAN_AddTxMessage(&hcan, &motors[motor].hdr, motors[motor].data, &motors[motor].mailbox);

	can_addr++;
}

/* USER CODE END 1 */
