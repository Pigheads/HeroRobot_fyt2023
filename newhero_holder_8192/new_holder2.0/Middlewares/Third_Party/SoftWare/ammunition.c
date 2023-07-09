/**
  ******************************************************************************
  * @file    
  * @author  ycz，lj
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
#include "ammunition.h"
#include "frictiongear.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include "can.h"
#include "keyboard.h"
#include "judge.h"
#include "mathfun.h"
#include "chassis.h"
#include "vision.h"

/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define Speed_Fix 1300

/* variables -----------------------------------------------------------------*/
Ammunition_t Ammunition;
uint8_t temp_block=0;
uint8_t state=0;
uint8_t temp_reverse=0;
uint8_t temp_empty=0;
uint8_t flag=0;
/* function ------------------------------------------------------------------*/

int test=0;
int Heat_Prediction(uint16_t Q1,uint16_t Q0,uint16_t Qs)
{	 
	 int16_t Max_HP=JUDGE_u16GetMaxHP();
	test = Max_HP;
	 if(Max_HP<0)
		 return 0;
	 int16_t Remain_HP=JUDGE_u16GetRemainHP();
	 if(Q1<=Q0)
		 return 0;
	 else
	 {
    while(Q1>Q0)		 
	  {
		 Remain_HP -= ((Q1-Q0)/250)/10*Max_HP;
		 Q1=Q1-Qs/10;
	  }
	  if(Remain_HP>=0.4*Max_HP)
		 return 0;
	  else
		 return 1;
	 }
}

void HeatControl(void)
{
	 uint16_t Q1=JUDGE_u16GetRemoteHeat42();
	 uint16_t Q0=JUDGE_u16GetHeatLimit();
	 uint16_t Qs=JUDGE_u16GetHeatRate();
	 if(Control_state.overheat.cr_state==1)
	 {
     if(Q1+100>=2*Q0||Heat_Prediction(Q1+100,Q0,Qs))
	   {
       Control_state.HeatPermit=1;
	   }
		 else
			 Control_state.HeatPermit=0;
	 }
	 else
	 {
		 if(Q1+100>Q0)
			 Control_state.HeatPermit=1;
		 else
			 Control_state.HeatPermit=0;
	 }
}

uint8_t Ammunition_State(uint8_t input)
{
   switch(state)
   {
		 case 0: 
			if(input) 
				state =1;
		 break;
		 case 1:
			if(input) 
				state =2;
		  else 
				state =0;
		 break;
		 case 2:  
      if(input)
     		state =2;
      else
        state =3;	
		 break;
     case	3:		
			if(input)
				state =2;
			else
			{
				state =0;
				flag=1;
			}
		 break;
	 }
    return flag;	 
}

void Ammunition_protect(void)
{
	flag=0;
}

void ShootEmpty_detection(void)
{
	if(Control_state.ShootEmpty)
	{
		Control_state.shoot_or_not = 0;
		Control_state.block_and_reverse = 0;	
		flag=0;
	}
}
void Ammunition_re_control(void)
{
		if(Control_state.Stop_reverse==1)
			 {
					Control_state.ShootBlock = 0;
					Control_state.block_and_reverse = 0;
					Control_state.shoot_or_not = 0;
			 }
		if((Control_state.SingleShoot==0)&&(Control_state.HeatPermit==0)&&(Control_state.ShootBlock==0)&&(Control_state.Shootpermit==1))
		{
			//防抖 得到flag
			Ammunition_State(Control_state.Remote_Wheel>=330||Control_state.Remote_Wheel<=-330);
			if((abs(vData.Pos.y) <= 1 && abs(vData.Pos.x) <= 1))
				Control_state.shoot_or_not = 1;
//				flag = 1;
//			else 
//				flag = 0;
			if(flag==1)
				 Control_state.shoot_or_not = 1;                   //可以发弹 
			 else
				 Control_state.shoot_or_not = 0;                   //不能发弹 
		}
		else if(Control_state.ShootBlock==1)
	  {
			 temp_reverse++;
			 if(temp_reverse<=70)
			 {
				 Control_state.block_and_reverse = 1;							//反转
			 }
			 else
			 {
				 temp_reverse=0;
				 flag=0;
				 Control_state.ShootBlock=0;
				 Control_state.block_and_reverse = 0;
			 }
	  }
		else
		{
			Ammunition_protect();
		}
}
void Ammunition_kb_control(void)
{
		if((Control_state.SingleShoot==0)&&(Control_state.Shootpermit==1))
		{ 
			if(Control_state.Remote_S1!=2)
			{
				Ammunition_State(Control_state.ammunition);				 //防抖 得到flag
				//get flag
			}
			else
			{
//				Ammunition_State(Control_state.Remote_Wheel>=330||Control_state.Remote_Wheel<=-330||(abs(vData.Pos.y) <= 0.5 && abs(vData.Pos.x) <= 0.5));
				Ammunition_State(Control_state.Remote_Wheel>=330||Control_state.Remote_Wheel<=-330);
					//get flag
			}
		}
		else
		{
			Ammunition_protect();
		}
}

void Ammunition_GetMoveData(void)
{
			if(Observer.Tx.DR16_Rate>15)
			{	
				Ammunition_kb_control();
			}
			else
			{
				Ammunition_protect();
				flag=0;
			}
}

void Ammunition_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)				
    {
			Chassis.CanData[6] = flag;
    }   
    else
    {
				memset(Chassis.CanData,0,sizeof(Chassis.CanData));
    }
}

void Ammunition_Process(void)
{
    Ammunition_GetMoveData();
    Ammunition_CanTransmit();
		//Chassis.CanData[6]=Control_state.shoot_or_not|(Control_state.block_and_reverse<<1)
		//can2反馈报文ID为0x500 和底盘三相速度数据一起发送
}	


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
