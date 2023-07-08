/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "holder.h"
#include "chassis.h"
#include "tim.h"
#include "ammunition.h"
#include "frictiongear.h"
#include "mathfun.h"
#include "keyboard.h"

#define F407_CAN_ID 0x200
#define CAN1_FIFO CAN_RX_FIFO0
#define CAN2_FIFO CAN_RX_FIFO0

uint16_t Angle_Rx_yaw=0;
uint16_t Angle_Rx_ammunition;
Mcircle_t MC_ammunition;
uint8_t YawData[]={0};
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = CAN2_RX_Pin|CAN2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, CAN1_RX_Pin|CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN2_RX_Pin|CAN2_TX_Pin);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 0;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN1_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan1,&fcan);
    HAL_CAN_Start(&hcan1);
    
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 0;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN2_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan2,&fcan);
    HAL_CAN_Start(&hcan2);
    
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN1_Transmit(uint16_t ID,uint8_t *pData)
{
    CAN_TxHeaderTypeDef TxMsg;
		int32_t MailBox;
    TxMsg.StdId = ID;
    TxMsg.IDE = CAN_ID_STD;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 8;
    Observer.Rx.CAN1_Tx_Rate++;
	  HAL_CAN_AddTxMessage(&hcan1,&TxMsg,pData,&MailBox); 
}

void CAN2_Transmit(uint16_t ID,uint8_t *pData)
{
    CAN_TxHeaderTypeDef TxMsg;
		int32_t MailBox;
    TxMsg.StdId = ID;
    TxMsg.IDE = CAN_ID_STD;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 8;
    Observer.Rx.CAN2_Tx_Rate++;
	  HAL_CAN_AddTxMessage(&hcan2,&TxMsg,pData,&MailBox); 
}

int16_t temp=0; 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    UNUSED(hcan);
    uint8_t Data[8];
		
    CAN_RxHeaderTypeDef RxMsg;
    if(hcan->Instance == CAN1) /* CAN1 Receive DATA yawID:0x209 pitchID: 0x208 */
    {
        HAL_CAN_GetRxMessage(hcan, CAN1_FIFO, &RxMsg, Data);
        switch(RxMsg.StdId)
        {
					 case 0x201:
                Observer.Rx.CAN_0x201_Rate++;
                Chassis.M3508[0].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[0].Rx.Current = Data[4] << 8 | Data[5];
            break;
						
            case 0x202:
                Observer.Rx.CAN_0x202_Rate++;
                Chassis.M3508[1].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[1].Rx.Current = Data[4] << 8 | Data[5];
            break;
						
            case 0x203:
                Observer.Rx.CAN_0x203_Rate++;
                Chassis.M3508[2].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[2].Rx.Current = Data[4] << 8 | Data[5];
            break;
						
            case 0x204:
                Observer.Rx.CAN_0x204_Rate++;
                Chassis.M3508[3].Rx.Speed   = Data[2] << 8 | Data[3];
                Chassis.M3508[3].Rx.Current = Data[4] << 8 | Data[5];
            break;
							case 0x206:
                Observer.Rx.CAN_0x206_Rate++;
								Angle_Rx_ammunition         = Data[0] << 8 | Data[1];
								Ammunition.RxSpeed          = Data[2] << 8 | Data[3];
						    Ammunition.RxCurrent        = Data[4] << 8 | Data[5];
						
						    Circle_Continue(&MC_ammunition,Angle_Rx_ammunition);
						    Ammunition.RxAngle  = Angle_Rx_ammunition/10 +(MC_ammunition.Circle*819);
						    Ammunition.RxSpeed_diff     = diff(Ammunition.RxAngle);
						break;	
						case 0x20B:
                Observer.Rx.CAN_0x209_Rate++;
						    Angle_Rx_yaw=360-(Data[0] << 8 | Data[1])*360/8192;
								//偏航电机（云台）相对底盘的角度
						    if(Angle_Rx_yaw>=(306-204))
									Angle_Rx_yaw=Angle_Rx_yaw-(306-204);
								else
									Angle_Rx_yaw=Angle_Rx_yaw+54+204;
  						  Holder.Yaw._0x209.Rx.Angle=Angle_Rx_yaw;
                //偏航电机的速度
								Holder.Yaw._0x209.Rx.Speed = Data[2] << 8 | Data[3];
								YawData[0] = (uint8_t)(Angle_Rx_yaw >> 8);
								YawData[1] = (uint8_t)(Angle_Rx_yaw);
								YawData[2] = (uint8_t)Data[2];
								YawData[3] = (uint8_t)Data[3];
								YawData[4] = (uint8_t)(Angle_Rx_yaw >> 8);
								YawData[5] = (uint8_t)(Angle_Rx_yaw);
								YawData[6] = (uint8_t)(Angle_Rx_yaw >> 8);
								YawData[7] = (uint8_t)(Angle_Rx_yaw);
								CAN2_Transmit(0x400,YawData);
						break;		
						case 0x211:
							  Chassis.SuperCap.state = (int16_t)Data[0];
								//电容电压
								Chassis.SuperCap.cap_vol = (int16_t)Data[1] << 8 | Data[2];
								//底盘功率
								Chassis.SuperCap.chassis_power = Data[3] << 24 | Data[4]<<16 | Data[5]<<8 | Data[6];
						break;		
        }
    }
    
    if(hcan->Instance == CAN2)/* CAN2 Receive DATA ID:0x201 0x202 0x203 0x204 */
    {		
				uint8_t CAN2_buf[8]={0};
        HAL_CAN_GetRxMessage(hcan, CAN2_FIFO, &RxMsg, CAN2_buf);
        switch(RxMsg.StdId)
        {
						
//						case 0x206:
//                Observer.Rx.CAN_0x206_Rate++;
//								Angle_Rx_ammunition         = Data[0] << 8 | Data[1];
//								Ammunition.RxSpeed          = Data[2] << 8 | Data[3];
//						    Ammunition.RxCurrent        = Data[4] << 8 | Data[5];
//						
//						    Circle_Continue(&MC_ammunition,Angle_Rx_ammunition);
//						    Ammunition.RxAngle  = Angle_Rx_ammunition/10 +(MC_ammunition.Circle*819);
//						    Ammunition.RxSpeed_diff     = diff(Ammunition.RxAngle);
//						break;		
						case 0x500:
								Chassis.MoveData.Front				= (int16_t)CAN2_buf[0] << 8 | (int16_t)CAN2_buf[1];
								Chassis.MoveData.Right 				= (int16_t)CAN2_buf[2] << 8 | (int16_t)CAN2_buf[3];
								Chassis.MoveData.ClockWise		= (int16_t)CAN2_buf[4] << 8 | (int16_t)CAN2_buf[5];
								flag													= (uint8_t)(CAN2_buf[6]);
								Observer.Tx.DR16_Rate 				= (int16_t)CAN2_buf[7];
								break;
						
						case 0x600:
								Holder.Yaw ._0x209.OutputLpf	= (int16_t)CAN2_buf[0] << 8 | (int16_t)CAN2_buf[1];
//								Holder.Yaw._0x209.TarAngle		= (int16_t)CAN2_buf[2] << 8 | (int16_t)CAN2_buf[3];
//								Holder.Yaw.Rx.Speed						= (int16_t)CAN2_buf[4] << 8 | (int16_t)CAN2_buf[5];
								Control_state.Remote_Wheel		= (int16_t)CAN2_buf[6] << 8 | (int16_t)CAN2_buf[7];
								break;
						case 0x700:
								Control_state.open_cap.cr_state = CAN2_buf[0];
								break;
						
        }
    }
}
/*

*/

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
