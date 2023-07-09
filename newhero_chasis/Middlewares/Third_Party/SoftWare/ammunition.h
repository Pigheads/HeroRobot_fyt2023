/**
  ******************************************************************************
  * @file    
  * @author  ycz,lj
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
#ifndef _Ammunition_H
#define _Ammunition_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
/* typedef -------------------------------------------------------------------*/;

typedef struct _Ammunition_t
{
	  M3508_t _0x206;
	  int16_t TarCurrent;
	  int16_t RxCurrent;
	  int16_t RxCurrentLPf;
	  int16_t TarSpeed;
	  int16_t RxSpeed;
	  int16_t RxSpeedLPf;
	  int32_t RxSpeed_diff;
	  int32_t RxAngle;
	  int32_t RxAngleLPf;
	  int32_t TarAngle;
    PID_AntiIntegralType PidAngle_A;
	  PID_AntiIntegralType PidSpeed_A;
	  
	  PID_IncrementType PidSpeed;
	  PID_IncrementType PidCurrent;
		uint8_t CanData2[8];
		uint8_t ShootFlag[8];
}Ammunition_t;

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
extern Ammunition_t Ammunition;

/* function ------------------------------------------------------------------*/
void  Ammunition_PidInit(void);
void Ammunition_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
