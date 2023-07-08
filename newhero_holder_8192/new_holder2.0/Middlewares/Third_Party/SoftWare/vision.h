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
#include "kfilter.h"
#include "mathfun.h"

#define VISION_RECEIVE_DATA_SIZE (200) // �������ݴ�С
#define MAF_MaxSize 100

#define CAMERA_X_DEVIATION	-0.175f			//������������������ƫ�������������ϡ���ǰΪ��
#define CAMERA_Y_DEVIATION 	0.055f
#define CAMERA_Z_DEVIATION 	0.0f
/* typedef -------------------------------------------------------------------*/
typedef struct
{
    float x;
    float y;
    float z;
} vPos_t;
// x ����Ŀ��Ƕȣ�y ƫ��Ŀ��Ƕȣ�z ���� ��λm

typedef struct moving_Average_Filter
{
	float num[MAF_MaxSize];
	uint8_t lenth;
	uint8_t pot;//��ǰλ��
	float total;
	float aver_num;
}moving_Average_Filter;	//�������MAF_MaxSize��


typedef struct
{
    vPos_t Pos;
    uint8_t index;
    uint8_t buf[200];
		int16_t time_interval;
		int32_t cur_gyro_yaw, cur_gyro_pitch;
}vData_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern vData_t vData;
/* function ------------------------------------------------------------------*/
void vision_init(void);
void Vision_RecvData(uint8_t byte);
void Vision_SendData(void);
void average_init(moving_Average_Filter *Aver, uint8_t lenth);
void average_add(moving_Average_Filter *Aver, float add_data);
float average_get(moving_Average_Filter *Aver, uint16_t pre);
void Holder_CoordinateTransform(void);
void vision_predict(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
