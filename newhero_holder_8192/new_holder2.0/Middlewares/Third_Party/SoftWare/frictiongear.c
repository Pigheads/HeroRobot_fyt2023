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
#include "frictiongear.h"
#include "tim.h"
#include <string.h>
#include "can.h"
#include "keyboard.h"
#include "judge_rx.h"
#include "mathfun.h"
#include <math.h>
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define Speed_16m 5900//6550
#define Speed_10m 4000
//#define Speed_16m 4700
//#define Speed_10m 4200

/* variables -----------------------------------------------------------------*/
Frictiongear_t Frictiongear;
uint16_t shootspeed_10;
uint16_t shootspeed_16;

/* function ------------------------------------------------------------------*/
void Friction_protect(void);
void Frictiongear_PidInit(void)
{
			pid_init_increment(&Frictiongear.PidSpeed1,5.2,0.35,1.9,999,16000);
	
			pid_init_increment(&Frictiongear.PidSpeed2,5.2,0.35,1.9,999,16000);
}

void Friction_re_control(void)
{
		 switch(Control_state.Remote_S2)
		  {					
			  case 1:                                                //摩擦轮不转
					 Frictiongear ._0x201.TarSpeed=0;
					 Frictiongear ._0x202.TarSpeed=0;
				   Control_state.Shootpermit=0;
					 Friction_protect();
			  break;
			  case 2: 
					 Control_state.Shootpermit=1;
					if(JUDGE_u16GetSpeedHeat42Limit()==10)
					{
					      Frictiongear ._0x201.TarSpeed=-Speed_10m;
			   				Frictiongear ._0x202.TarSpeed=+Speed_10m;
					}
					else if(JUDGE_u16GetSpeedHeat42Limit()==16)
					{
			          Frictiongear ._0x201.TarSpeed=-Speed_16m;
						  	Frictiongear ._0x202.TarSpeed=+Speed_16m;
					}
					else
					{
						    Frictiongear ._0x201.TarSpeed=-Speed_16m;
			   				Frictiongear ._0x202.TarSpeed=+Speed_16m;
					}
//					 Frictiongear ._0x201.TarSpeed=-Speed_16m;
//			   	 Frictiongear ._0x202.TarSpeed=+Speed_16m;
//					 Frictiongear ._0x201.TarSpeed=0;
//					 Frictiongear ._0x202.TarSpeed=0;
//				   Control_state.Shootpermit=0;
//					 Friction_protect();
        break;
				case 3:
					 Control_state.Shootpermit=1;
					if(JUDGE_u16GetSpeedHeat42Limit()==10)
					{
					      Frictiongear ._0x201.TarSpeed=-Speed_10m;
			   				Frictiongear ._0x202.TarSpeed=+Speed_10m;
					}
					else if(JUDGE_u16GetSpeedHeat42Limit()==16)
					{
			          Frictiongear ._0x201.TarSpeed=-Speed_16m;
						  	Frictiongear ._0x202.TarSpeed=+Speed_16m;
					}
					else
					{
						    Frictiongear ._0x201.TarSpeed=-Speed_16m;
			   				Frictiongear ._0x202.TarSpeed=+Speed_16m;
					}
//			     Frictiongear ._0x201.TarSpeed=-Speed_16m;
//					 Frictiongear ._0x202.TarSpeed=+Speed_16m;
        break;
			}
}

void Frictiongear_LPfIn(void)
{
	   Frictiongear._0x201.Rx.SpeedLpf=Receive_filter_4(Frictiongear._0x201.Rx.Speed,Frictiongear.Filter_Rx_Buf1);
	   Frictiongear._0x202.Rx.SpeedLpf=Receive_filter_4(Frictiongear._0x202.Rx.Speed,Frictiongear.Filter_Rx_Buf2);
	   Frictiongear.Rxspeed=-Frictiongear._0x202.Rx.SpeedLpf;
}

void Friction_speed_control(void)
{
	switch(Control_state.shoot_speed.cr_state)
	{
		case 0:
			shootspeed_10 = Speed_10m;
		  shootspeed_16 = Speed_16m;
		break;
		case 1:
		  shootspeed_10 = Speed_10m-50;
		  shootspeed_16 = Speed_16m-50;
		break;
		case 2:
			shootspeed_10 = Speed_10m+50;
		  shootspeed_16 = Speed_16m+50;			
		break;
	}
}

void Friction_kb_control(void)
{
			switch(Control_state.shoot_lock.cr_state)
		  {		
			  case 0: 
					 Frictiongear ._0x201.TarSpeed=-shootspeed_16;
				   Frictiongear ._0x202.TarSpeed=+shootspeed_16;
				   Control_state.Shootpermit=1;
			  break;
			  case 1: 
					 Control_state.Shootpermit=1;
				  if(JUDGE_u16GetSpeedHeat42Limit()==10)
					{
					      Frictiongear ._0x201.TarSpeed=-shootspeed_10;
			   				Frictiongear ._0x202.TarSpeed=+shootspeed_10;
					}
					else if(JUDGE_u16GetSpeedHeat42Limit()==16)
					{
			          Frictiongear ._0x201.TarSpeed=-shootspeed_16;
						  	Frictiongear ._0x202.TarSpeed=+shootspeed_16;
					}
					else
					{
						    Frictiongear ._0x201.TarSpeed=-shootspeed_16;
			   				Frictiongear ._0x202.TarSpeed=+shootspeed_16;
					}
				break;
			 }	
}

void Friction_protect(void)
{
		Frictiongear ._0x201.TarSpeed = Frictiongear ._0x201.Rx.Speed;
		Frictiongear ._0x202.TarSpeed = Frictiongear ._0x202.Rx.Speed;	
		pid_increment_update(0,0,&Frictiongear.PidSpeed1);
		pid_increment_update(0,0,&Frictiongear.PidSpeed2);
		Frictiongear .PidSpeed1.ctrOut =0;
		Frictiongear .PidSpeed2.ctrOut =0;
		Frictiongear ._0x201.Output = 0;
		Frictiongear ._0x202.Output = 0;
}

void Frictiongear_GetMoveData(void)
{
			if(Observer.Tx.DR16_Rate>15)	 
		  {
			 if(Control_state.Remote_S1!=2)
			  Friction_kb_control();
			 else
			  Friction_re_control();
		  }
			else
		  {
        Friction_protect();
		  }
			if(Frictiongear._0x202.Rx.Speed<0.90*Frictiongear ._0x202.TarSpeed&&Frictiongear ._0x202.TarSpeed>1000)	
			{
				//因为现在摩擦轮的202给的速度是正的 201是负的 所以这里比较202的速度 
				//具体要根据实际摩擦轮的安装调整
				//检测到发弹 单发标志位置1
				Control_state.SingleShoot = 1;
				flag=0;
			}
			if(Control_state.SingleShoot==1&&Frictiongear._0x202.Rx.Speed>=0.95*Frictiongear ._0x202.TarSpeed)
			{	
				//已单发过 且摩擦轮速度恢复 可以继续单发
        Control_state.SingleShoot = 0;
			}
}

void Frictiongear_LPfOut(void)
{
				Frictiongear ._0x201 .OutputLpf =  0.75*Frictiongear ._0x201 .Output + 0.25 * Frictiongear ._0x201 .OutputLpf;
        Frictiongear ._0x202 .OutputLpf =  0.75*Frictiongear ._0x202 .Output + 0.25 * Frictiongear ._0x202 .OutputLpf;
}
int Rx01,Rx02;
void Frictiongear_PidRun(void)
{
				Frictiongear ._0x201 .Output = pid_increment_update(Frictiongear ._0x201.TarSpeed, Frictiongear._0x201.Rx.SpeedLpf , &Frictiongear .PidSpeed1);
				Frictiongear ._0x202 .Output = pid_increment_update(Frictiongear ._0x202.TarSpeed, Frictiongear._0x202.Rx.SpeedLpf , &Frictiongear .PidSpeed2);
		Rx01=Frictiongear._0x201.Rx.Speed;
		Rx02=Frictiongear._0x202.Rx.Speed;
}
//uint8_t jjjj = 0;
void Frictiongear_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>1)
    {
						Frictiongear.CanData1[0]=(uint8_t)((Frictiongear._0x201.OutputLpf)>>8);
            Frictiongear.CanData1[1]=(uint8_t)(Frictiongear._0x201.OutputLpf);
            Frictiongear.CanData1[2]=(uint8_t)((Frictiongear._0x202.OutputLpf)>>8);
            Frictiongear.CanData1[3]=(uint8_t)(Frictiongear._0x202.OutputLpf);
    }   
    else
    {
				memset(Frictiongear.CanData1,0,sizeof(Frictiongear.CanData1));  
    }
//		
}

void Frictiongear_Process(void)
{
	  Frictiongear_LPfIn();
	  Friction_speed_control();
    Frictiongear_GetMoveData();
    if(Observer.Tx.DR16_Rate>1) 
			Frictiongear_PidRun();
    Frictiongear_LPfOut();
    Frictiongear_CanTransmit();
}



/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
