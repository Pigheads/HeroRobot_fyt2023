/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief   
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _KEY_H
#define _KEY_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "gpio.h"
/* typedef -------------------------------------------------------------------*/
typedef enum _Key_e
{
    Up,
    Down,
    Back,
    Enter,
    Max,
}Key_e;

typedef struct _KeyMsg_t
{
    Key_e Key;
    uint8_t Status;
}KeyMsg_t;

/* define --------------------------------------------------------------------*/
//#define USE_ADC_OLED

#if defined USE_ADC_OLED

#define KEY_UP		  ADC_ReadVol(0.0f)	
#define KEY_DOWN		ADC_ReadVol(2.4f)	
#define KEY_BACK		ADC_ReadVol(1.7f)	
#define KEY_ENTER		ADC_ReadVol(0.63f)	

#else

#define KEY_UP      HAL_GPIO_ReadPin(GPIOD,KeyUp_Pin)
#define KEY_DOWN    HAL_GPIO_ReadPin(GPIOD,KeyDown_Pin)
#define KEY_BACK    HAL_GPIO_ReadPin(GPIOD,KeyBack_Pin)
#define KEY_ENTER   HAL_GPIO_ReadPin(GPIOD,KeyEnter_Pin)

#endif

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
void Key_Process(void);
uint8_t GetKeyMsgFromFifo(KeyMsg_t *key_msg);

#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
