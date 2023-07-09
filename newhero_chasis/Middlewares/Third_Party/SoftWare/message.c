/**
  ******************************************************************************
  * @file    
  * @author  ycz,sy
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
#include "message.h"
#include "imu_data_decode.h"
#include "holder.h"
#include "motor.h"
#include "usart.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
static Mcircle_t mc_imu_yaw = {0};
static float Eular[3] = {0};
static int16_t Gyro[3] = {0};
/* function ------------------------------------------------------------------*/

void RemoteDataMsg_Process(RemoteData_t *RDMsg)
{
    RDMsg->Ch0  = (  (int16_t) uart1_buf[0]       | ( (int16_t) uart1_buf[1]  << 8 )) & 0x07FF;
    RDMsg->Ch0 -= 1024;
    RDMsg->Ch1  = (( (int16_t) uart1_buf[1] >> 3) | ( (int16_t) uart1_buf[2]  << 5 )) & 0x07FF;
    RDMsg->Ch1 -= 1024;
    RDMsg->Ch2  = (( (int16_t) uart1_buf[2] >> 6) | ( (int16_t) uart1_buf[3]  << 2 )  
                                                  | ( (int16_t) uart1_buf[4]  << 10)) & 0x07FF;
    RDMsg->Ch2 -= 1024;
    RDMsg->Ch3  = (( (int16_t) uart1_buf[4] >> 1) | ( (int16_t) uart1_buf[5]  << 7 )) & 0x07FF;
    RDMsg->Ch3 -= 1024;
    RDMsg->S1   = (            uart1_buf[5] >> 6)                                     & 0x03;
    RDMsg->S2   = (            uart1_buf[5] >> 4)                                     & 0x03;
         
    RDMsg->Mouse_x = ( (int16_t) uart1_buf[6] | (int16_t) uart1_buf[7] << 8);
    RDMsg->Mouse_y = ( (int16_t) uart1_buf[8] | (int16_t) uart1_buf[9] << 8);
    if(RDMsg->Mouse_x>=25)
			RDMsg->Mouse_x=25;
		if(RDMsg->Mouse_x<=-25)
			RDMsg->Mouse_x=-25;   
    RDMsg->MouseClick_left  = uart1_buf[12];
    RDMsg->MouseClick_right = uart1_buf[13]; 
      
    RDMsg->Key     = ( (int16_t) uart1_buf[14] |   (int16_t) uart1_buf[15] << 8 );
//    RDMsg->Wheel   = ( (int16_t) uart1_buf[16] | ( (int16_t) uart1_buf[17] << 8 )) & 0x07FF;
//    RDMsg->Wheel   = -RDMsg->Wheel + 1024;
		
    RDMsg->KeyBoard.w     =  RDMsg->Key & KEY_PRESSED_OFFSET_W;
    RDMsg->KeyBoard.s     = (RDMsg->Key & KEY_PRESSED_OFFSET_S)>>1;
    RDMsg->KeyBoard.a     = (RDMsg->Key & KEY_PRESSED_OFFSET_A)>>2;
    RDMsg->KeyBoard.d     = (RDMsg->Key & KEY_PRESSED_OFFSET_D)>>3;
    RDMsg->KeyBoard.shift = (RDMsg->Key & KEY_PRESSED_OFFSET_SHIFT)>>4;
    RDMsg->KeyBoard.ctrl  = (RDMsg->Key & KEY_PRESSED_OFFSET_CTRL)>>5;
    RDMsg->KeyBoard.q     = (RDMsg->Key & KEY_PRESSED_OFFSET_Q)>>6;
    RDMsg->KeyBoard.e     = (RDMsg->Key & KEY_PRESSED_OFFSET_E)>>7;
    RDMsg->KeyBoard.r     = (RDMsg->Key & KEY_PRESSED_OFFSET_R)>>8;
    RDMsg->KeyBoard.f     = (RDMsg->Key & KEY_PRESSED_OFFSET_F)>>9;
    RDMsg->KeyBoard.g     = (RDMsg->Key & KEY_PRESSED_OFFSET_G)>>10;
    RDMsg->KeyBoard.z     = (RDMsg->Key & KEY_PRESSED_OFFSET_Z)>>11;
    RDMsg->KeyBoard.x     = (RDMsg->Key & KEY_PRESSED_OFFSET_X)>>12;
    RDMsg->KeyBoard.c     = (RDMsg->Key & KEY_PRESSED_OFFSET_C)>>13;
    RDMsg->KeyBoard.v     = (RDMsg->Key & KEY_PRESSED_OFFSET_V)>>14;
    RDMsg->KeyBoard.b     = (RDMsg->Key & KEY_PRESSED_OFFSET_B)>>15; 
}

static float IMUAngle_Continue(float imu_angle)
{
	float out_data = 0; 

	Circle_Continue(&mc_imu_yaw, (imu_angle + 180.0f) * 22.7555556f);
	out_data = 819 * (float)(mc_imu_yaw.Circle + (float)(mc_imu_yaw.Angle) / 8192);

	return out_data;    
}

void HolderDataMsg_Process(HolderData_t *HDMsg)
{
    get_eular(Eular);
    HDMsg->Angle   = (int16_t)IMUAngle_Continue(Eular[2]);
    HDMsg->PxAngle = (int16_t)Eular[1];
    get_raw_gyo(Gyro);
    HDMsg->Speed    =  -Gyro[2]/40;
	  HDMsg->PxSpeed  =   Gyro[1]/50;
	  HDMsg->RxAngle=Holder.Yaw._0x209.Rx.Angle;
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
