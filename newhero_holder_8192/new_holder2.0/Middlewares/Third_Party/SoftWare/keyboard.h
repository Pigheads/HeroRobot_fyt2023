/**
  ******************************************************************************
  * @file    
  * @author  lj
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
#ifndef _KEYBOARD_H
#define _KEYBOARD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "message.h"

/* typedef -------------------------------------------------------------------*/
typedef enum 
{
  KEY_Q = 0,
  KEY_E,
	KEY_Z,
	KEY_C
}KEY_PIN;

typedef struct _switch_able_t
{
	uint8_t state_amount;
	uint8_t pin_state;
	uint8_t cr_state;
	KEY_PIN key;
}switchable_t;

typedef struct _robot_state_t
{
  uint8_t  direction[4];
	int16_t  running_speed[4];
	uint8_t  speed_up;
	uint8_t  ammunition;
	uint8_t  SingleShoot;
	uint8_t  HeatPermit;
	uint8_t  ShootBlock;
	uint8_t	 ShootEmpty;
	uint8_t  vision;
	uint8_t  Shootpermit;
	uint8_t  power_state;
	uint8_t  shift;
	int16_t  yaw_angle;
	int16_t  pitch_angle;
	uint8_t  vision_state;
	int16_t  gyoyaw_angle;
	
	switchable_t spin_state;
	switchable_t fast_stop;
	switchable_t spin_move;
  switchable_t shoot_lock;
	switchable_t shoot_speed;
	switchable_t aim_state;
	switchable_t overheat;
	switchable_t angle_q;
  switchable_t angle_e;
	switchable_t angle_z;
	switchable_t angle_c;
	switchable_t angle_f;
	//超级电容开启标志位
	switchable_t open_cap;
	
	uint8_t Remote_S1;
	uint8_t Remote_S2;
	int16_t Remote_Ch1;
	int16_t Remote_Wheel;
	
	uint8_t shoot_or_not;                     //是否允许底盘发弹的标志位
	uint8_t block_and_reverse;								//卡弹并且反转的标志位
	uint8_t Stop_reverse;
}state_t;

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
extern state_t Control_state;
/* function ------------------------------------------------------------------*/
void keyboard_controlinit(void);
void keyboard_state_judge(RemoteData_t RDMsg,HolderData_t HDMsg);
void state_switch(switchable_t *switchable,uint8_t input_pin);
void Angle_adjust(switchable_t *switchable,uint8_t input_pin);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
