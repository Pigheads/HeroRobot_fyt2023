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
#ifndef _Frictiongear_H
#define _Frictiongear_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
/* typedef -------------------------------------------------------------------*/;

typedef struct _Frictiongear_t
{
	  M3508_t _0x201;
		M3508_t _0x202;
    PID_IncrementType PidSpeed1;
    PID_IncrementType PidSpeed2;
    PID_IncrementType PidCurrent1;
    PID_IncrementType PidCurrent2;
	  int16_t Filter_Rx_Buf1[4];
    int16_t Filter_Rx_Buf2[4];	  
		uint8_t CanData1[8];
	  int16_t Rxspeed;
}Frictiongear_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Frictiongear_t Frictiongear;
extern uint8_t flag;
/* function ------------------------------------------------------------------*/
void Frictiongear_PidInit(void);
void Frictiongear_Process(void);


#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
