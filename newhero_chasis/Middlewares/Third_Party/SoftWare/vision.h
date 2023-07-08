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
#ifndef _VISION_H
#define _VISION_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "usart.h"
#include "message.h"

#define VISION_RECEIVE_DATA_SIZE (200) // 接收数据大小
/* typedef -------------------------------------------------------------------*/
typedef struct
{
    float x;
    float y;
    float z;
} vPos_t;
// x 偏航目标角度；y 俯仰目标角度；z 距离 单位m

typedef struct
{
    vPos_t Pos;
    uint8_t index;
    uint8_t buf[200];
}vData_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern vData_t vData;
/* function ------------------------------------------------------------------*/
void Vision_InitData(void);
void Vision_RecvData(uint8_t byte);
void Vision_SendData(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
