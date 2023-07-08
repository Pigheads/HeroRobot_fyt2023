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
#ifndef _HOLDER_H
#define _HOLDER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _Yaw_t
{
    GM6020_t _0x209;
    HolderData_t Rx;
    PID_AntiIntegralType PidSpeed_Y;
    PID_AntiIntegralType PidAngle_Y;
	  
//		PID_AbsoluteType PidSpeed;
    PID_AntiIntegralType PidSpeed;
	  PID_AbsoluteType_ThreeSection PIDAngle;
	  int16_t Filter_Rx_Buf[4];
	  uint8_t CanData[8];
}Yaw_t;

typedef struct _Pitch_t
{
    GM6020_t _0x20A;
	  GM6020_t _0x209;
	  int16_t  gyoSpeed;
	  int16_t  gyoAngle;
    PID_AntiIntegralType PidSpeed;
		PID_IncrementType PIDSpeed;
		PID_AbsoluteType PIDSPEED;
	
    PID_AntiIntegralType PidAngle;
	  PID_AbsoluteType_ThreeSection PIDAngle;
	  int16_t Filter_Rx_Buf[4];
	  uint8_t CanData[8];
}Pitch_t;

typedef struct _Holder_t
{
    Yaw_t Yaw;
    Pitch_t Pitch;
	  int16_t speed_limit;
	  int16_t speed;
}Holder_t;

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
extern Holder_t Holder;
extern int biaojiaojiaodu;
/* function ------------------------------------------------------------------*/
void Holder_GetMoveData(RemoteData_t RDMsg,HolderData_t HDMsg);
void Holder_PidInit(void);
void Holder_PidRun(void);
void Holder_LPfIn(void);
void Holder_LPfOut(void);
void Holder_CanTransmit(void);
void HolderMsg_Process(RemoteData_t RDMsg,HolderData_t HDMsg);

float Fall_measurement(float target_angle,float pitch_angle,float distance); //ÏÂ×¹²¹³¥
void Holder_PidRun_changed(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
