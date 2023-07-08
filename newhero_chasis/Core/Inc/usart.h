/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define CMD_WARE 3 
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART1_RemoteStart(void);
void UART3_JudgeStart(void);
void UART4_IMUStart(void);
void UART_SendWave(uint8_t num, uint8_t bytes, ...);
void UART6_VisionStart(void);
	
extern uint8_t uart1_buf[18];
extern uint8_t uart3_dma_flag;
extern uint8_t uart3_idle_flag;
extern uint8_t uart3_m0_buf[1024];
extern uint8_t uart3_m1_buf[1024];
extern uint8_t uart3_tx_buf[120];

uint8_t __HAL_DMA_GET_CURRMEMTAR(DMA_HandleTypeDef *hdma);
uint16_t __HAL_DMA_GET_CURRDATCOUT(DMA_HandleTypeDef *hdma);

extern DMA_HandleTypeDef hdma_usart3_rx;
extern uint8_t uart1_buf[18];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
