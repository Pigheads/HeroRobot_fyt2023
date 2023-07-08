/**
  ******************************************************************************
  * @file    
  * @author  sy
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "vision.h"
#include "string.h"
#include "judge.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
vData_t vData = {0};
/* function ------------------------------------------------------------------*/
void Vision_RecvData(uint8_t byte)
{
    vData.buf[vData.index] = byte;
    if (vData.buf[vData.index] == 0x03 && vData.buf[vData.index - 1] == 0xFC)
    {
        if (vData.buf[vData.index - 15] == 0X03 && vData.buf[vData.index - 14] == 0XFC)
        {
            memcpy(&vData.Pos, &vData.buf[vData.index - 13], 12);
            vData.index = 0;
        }
    }
    vData.index++;
    if (vData.index == 200)
        vData.index = 0;
}

uint8_t id;
void Vision_SendData(void)
{
    uint8_t tmp_data[6]; 
    static uint8_t color_state  = 0; 
    static uint8_t detect_state = 0;
    id = (uint8_t)JUDGE_u8GetRobotId();
    
    if (id <= 7 && id >0)
    {
        color_state = 1;
    }
    else if (id > 7)
    {
        color_state = 0;
    }
    
    switch(color_state)
    {
        case 0:
            tmp_data[2] = 0x00; /*< 红色*/
        break;

        case 1:
            tmp_data[2] = 0x01; /*< 蓝色*/
        break;
        
        default:
            tmp_data[2] = 0x00; /*< 红色*/
        break;
    }

    /* 帧头帧尾 */
    tmp_data[0] = 0x03;
    tmp_data[1] = 0xFC;

		tmp_data[3] = 0x00;
    tmp_data[4] = 0xFC;
    tmp_data[5] = 0x03;
    HAL_UART_Transmit_IT(&huart6, tmp_data, 6);
    HAL_Delay(100);
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
