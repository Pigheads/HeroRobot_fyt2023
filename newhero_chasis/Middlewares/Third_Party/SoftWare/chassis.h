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
#ifndef _CHASSIS_H
#define _CHASSIS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "motor.h"
#include "pid.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _MoveData_t
{
    int16_t Right;    
    int16_t Front;
    int16_t ClockWise;
		
}MoveData_t;
typedef struct _SuperCap
{
		uint8_t state;
		uint16_t in_vol;
		uint16_t cap_vol;
		uint32_t chassis_power;
		uint16_t in_cur;
		uint16_t power;
		uint16_t target_power;
		uint8_t can_data[8];
}SUPER_CAP_T;
typedef struct _Chassis_t
{
    M3508_t M3508[4];
    uint8_t CanData[8];
    MoveData_t MoveData;
		MoveData_t RampMoveData;
		SUPER_CAP_T SuperCap;
}Chassis_t;
typedef struct _PowerData_t
{   
	  int16_t InputVot;
	  int16_t CapVot;
	  int16_t Current;
	  int16_t TarPower;
	  
	  uint16_t Limit_power;
	  float Vehicle_power;
	  uint8_t CanData[8];
	  
	  uint16_t temPower;
}PowerData_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern Chassis_t Chassis;
extern PowerData_t Chassis_power;
extern uint8_t super_state;
/* function ------------------------------------------------------------------*/
void Chassis_PidInit(void);
void Chassis_Process(RemoteData_t RDMsg,HolderData_t HDMsg);
void Chassis_kb_control(int16_t RxAngle);
void Chassis_re_control(int16_t RxAngle,RemoteData_t RDMsg);
void Chassis_Protect(void);
void CanTranPower(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
