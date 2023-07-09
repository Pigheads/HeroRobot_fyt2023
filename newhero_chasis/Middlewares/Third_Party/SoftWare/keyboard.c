/**
  ******************************************************************************
  * @file
  * @author  
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
#include "chassis.h"
#include "holder.h"
#include "frictiongear.h"
#include "ammunition.h"
#include <string.h>
#include "keyboard.h"
#include "tim.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
state_t Control_state={0};
/* function ------------------------------------------------------------------*/
void keyboard_controlinit(void)
{
	Control_state.spin_state.state_amount=2;
	Control_state.spin_move.state_amount=2;
	Control_state.shoot_speed.state_amount=3;
	Control_state.shoot_lock.state_amount=2;
	Control_state.aim_state.state_amount=2;
	Control_state.open_cap.state_amount=2;
	
	Control_state.angle_q.key=KEY_Q;
	Control_state.angle_e.key=KEY_E;
	Control_state.angle_z.key=KEY_Z;
	Control_state.angle_c.key=KEY_C;
}

void keyboard_state_judge(RemoteData_t RDMsg,HolderData_t HDMsg)
{
 if(Observer.Tx.DR16_Rate>15)
 {
	state_switch(&Control_state.spin_state,RDMsg.KeyBoard.ctrl);
	state_switch(&Control_state.spin_move,RDMsg.KeyBoard.v);
	state_switch(&Control_state.shoot_lock,RDMsg.KeyBoard.g);
	state_switch(&Control_state.shoot_speed,RDMsg.KeyBoard.b);
	state_switch(&Control_state.aim_state,RDMsg.KeyBoard.r);
  Angle_adjust(&Control_state.angle_q,RDMsg.KeyBoard.q);
	Angle_adjust(&Control_state.angle_e,RDMsg.KeyBoard.e);
	Angle_adjust(&Control_state.angle_z,RDMsg.KeyBoard.z);
	Angle_adjust(&Control_state.angle_c,RDMsg.KeyBoard.c);
	//shiftżŞĆôłŹźśľçČÝ
//	state_switch(&Control_state.open_cap,RDMsg.KeyBoard.shift);	 
	 
	Control_state.direction[0]=RDMsg.KeyBoard.w;
	Control_state.direction[1]=RDMsg.KeyBoard.s;
	Control_state.direction[2]=RDMsg.KeyBoard.a;
	Control_state.direction[3]=RDMsg.KeyBoard.d;
	Control_state.ammunition  =RDMsg.MouseClick_left||RDMsg.KeyBoard.f;
	Control_state.vision      =RDMsg.MouseClick_right;
	Control_state.speed_up    =RDMsg.KeyBoard.shift;
	
	Control_state.Remote_Ch1  =RDMsg.Ch1;
	Control_state.Remote_S1   =RDMsg.S1;
	Control_state.Remote_S2   =RDMsg.S2;
  Control_state.Remote_Wheel=RDMsg.Wheel;
  Control_state.gyoyaw_angle=HDMsg.Angle;	
 }
 else
 {
	Control_state.direction[0]=0;
	Control_state.direction[1]=0;
	Control_state.direction[2]=0;
	Control_state.direction[3]=0;
	Control_state.ammunition  =0;
  Control_state.vision_state=0;
	
	Control_state.overheat.cr_state=0;
	Control_state.Shootpermit=0;
	Control_state.spin_state.cr_state=0;
	Control_state.spin_move.cr_state=0;
	Control_state.shoot_speed.cr_state=0;
	Control_state.shoot_lock.cr_state=0;
	Control_state.aim_state.cr_state=0;
 }
}

void state_switch(switchable_t *switchable,uint8_t input_pin)
{
	  switch(switchable->pin_state)
		{
			case 0:
				if(input_pin!=0)
					switchable->pin_state=1;
				break;
			case 1:
				if(input_pin==0)
					switchable->pin_state=0;
				else
					switchable->pin_state=2;
				break;
			case 2:
				if(input_pin==0)
				  switchable->pin_state=3;
        break;
      case 3:
        if(input_pin==0)
				{
					switchable->pin_state=0;
				  switchable->cr_state++;
				}
        else
          switchable->pin_state=2;
		}
      if(switchable->cr_state>=switchable->state_amount)
          switchable->cr_state=0;
}

void Angle_adjust(switchable_t *switchable,uint8_t input_pin)
{
	  switch(switchable->pin_state)
		{
			case 0:
				if(input_pin!=0)
					switchable->pin_state=1;
				break;
			case 1:
				if(input_pin==0)
					switchable->pin_state=0;
				else
					switchable->pin_state=2;
				break;
			case 2:
				if(input_pin==0)
				  switchable->pin_state=3;
        break;
      case 3:
        if(input_pin==0)
				{
					switchable->pin_state=0;
          switch(switchable->key)
					{
						case KEY_Q:
							Control_state.yaw_angle+=2;
						break;
						case KEY_E:
							Control_state.yaw_angle-=2;
						break;
						case KEY_Z:
							Control_state.pitch_angle-=6;
						break;
						case KEY_C:
							Control_state.pitch_angle+=6;
						break;
					}
				}
        else
          switchable->pin_state=2;
		}		
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
