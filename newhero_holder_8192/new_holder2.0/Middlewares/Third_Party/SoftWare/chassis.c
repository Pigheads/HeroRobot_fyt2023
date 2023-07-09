/**
  ******************************************************************************
  * @file
  * @author  ycz,lj
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
#include <string.h>
#include "tim.h"
#include "can.h"
#include "keyboard.h"
#include "judge_rx.h"
#include "mathfun.h"
#include "holder.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
#define Infantry_NORMAL   (0x01)
#define Infantry_SUPERCAP (0x00)
#define MAX 500
#define MAX_spin 300
#define MAX_fly 800 //600
#define ac_fly 15
#define ac 4 //12
/* variables -----------------------------------------------------------------*/
Chassis_t Chassis;
PowerData_t Chassis_power = {0};

int16_t  v_front=0;
int16_t  v_right=0;
static uint8_t super_state   = Infantry_NORMAL;
uint16_t speedlim = 7500;

/* function ------------------------------------------------------------------*/
double pp1=7.35,ii1=0.48,dd1=6.5;
double pp2=3.7,ii2=0.3,dd2=0.05;
void Chassis_PidInit(void)
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidSpeed,pp1,ii1,dd1,16000,16000);
				//pid_init_increment(&Chassis.M3508[i].PidSpeed,7.25,0.38,6.5,16000,16000);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.M3508[i].PidCurrent,pp2,ii2,dd2,16000,16000);
				//pid_init_increment(&Chassis.M3508[i].PidCurrent,0.9,0.2,0.05,16000,16000);
    }
}

void Chassis_LPfIn(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        Chassis.M3508[i].LPf.Speed = 0.75 * Chassis.M3508[i].Rx.Speed + 0.25 * Chassis.M3508[i].LPf.Speed;
    }
    for (i = 0; i < 4; i++)
    {
        Chassis.M3508[i].LPf.Current = 0.75 * Chassis.M3508[i].Rx.Current + 0.25 * Chassis.M3508[i].LPf.Current;
    }
}

void Chassis_GetMoveData(RemoteData_t RDMsg,HolderData_t HDMsg)
{
	HDMsg.RxAngle = Holder.Yaw._0x209.TarAngle;
	  if(Observer.Tx.DR16_Rate>15)
		{
			 if(Control_state.Remote_S1!=2)
					Chassis_kb_control(HDMsg.RxAngle);
			 else
					Chassis_re_control(HDMsg.RxAngle,RDMsg);
		}
	  else
		{
				Chassis_Protect();
		}
}

void Chassis_LPfOut(void)
{
    uint8_t i;
    for(i=0;i<4;i++) 
    {
      Chassis.M3508[i].OutputLpf = 0.75 * Chassis.M3508[i].Output + 0.25 * Chassis.M3508[i].OutputLpf;
			if(Chassis.M3508[i].OutputLpf>=16000)
				Chassis.M3508[i].OutputLpf=16000;
			if(Chassis.M3508[i].OutputLpf<=-16000)
				Chassis.M3508[i].OutputLpf=-16000;
    }
}	

void Chassis_PidRun(void)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {   
			  Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed,Chassis.M3508[i].LPf.Speed,&Chassis.M3508[i].PidSpeed);
		}    

    for (i = 0; i < 4; i++)
    {
        Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].TarCurrent,Chassis.M3508[i].LPf.Current,&Chassis.M3508[i].PidCurrent);
    }
}
int16_t Front,Right;
void Chassis_CanTransmit(void)
{
	Front = Chassis.MoveData.Front;
	Right = Chassis.MoveData.Right;
    if(Observer.Tx.DR16_Rate>15)
    {
				Chassis.CanData[0]=(uint8_t)(Chassis.MoveData.Front>>8);
				Chassis.CanData[1]=(uint8_t)(Chassis.MoveData.Front);
				Chassis.CanData[2]=(uint8_t)(Chassis.MoveData.Right>>8);
				Chassis.CanData[3]=(uint8_t)(Chassis.MoveData.Right);
				Chassis.CanData[4]=(uint8_t)(Chassis.MoveData.ClockWise>>8);
				Chassis.CanData[5]=(uint8_t)(Chassis.MoveData.ClockWise);
				Chassis.CanData[7]=(uint8_t)	Observer.Tx.DR16_Rate;		
				Chassis.Key_SUPERCAP[0]		= Control_state.open_cap.cr_state;   //shift开启超级电容
		}   
    else
    {
				memset(Chassis.CanData,0,sizeof(Chassis.CanData));
				memset(Chassis.Key_SUPERCAP,0,sizeof(Chassis.Key_SUPERCAP));
    }
}

void Chassis_Protect(void)
{
	 memset(Chassis.CanData,0,sizeof(Chassis.CanData));
	 memset(Chassis_power.CanData,0,sizeof(Chassis_power.CanData));
	 memset(Control_state.running_speed,0,sizeof(Control_state.running_speed));
	 for(int i=0;i<4;i++)
	 {
		 Chassis.M3508[i].TarSpeed = 0; 
		 Chassis.M3508[i].PidSpeed.errNow=0;
     Chassis.M3508[i].PidSpeed.errOld1=0;
		 Chassis.M3508[i].PidSpeed.errOld2=0;
		 Chassis.M3508[i].PidSpeed.ctrOut=0;
		 pid_increment_update(0,0,&Chassis.M3508[i].PidSpeed);
		 Chassis.M3508[i].TarCurrent = 0; 
		 Chassis.M3508[i].PidCurrent.errNow=0;
		 Chassis.M3508[i].PidCurrent.errOld1=0;
		 Chassis.M3508[i].PidCurrent.errOld2=0;
		 Chassis.M3508[i].PidCurrent.ctrOut=0;
		 pid_increment_update(0,0,&Chassis.M3508[i].PidCurrent);
	 }
}

void Chassis_PowerControl(void)
{ 
	  Chassis_power.Limit_power=JUDGE_u16GetChassisPowerLimit();
	  Chassis_power.Vehicle_power=JUDGE_f32GetChassisPower();

	 if(Chassis_power.Limit_power!=0&&Chassis_power.Limit_power<=120)
	  Chassis_power.temPower=Chassis_power.Limit_power*100;
	 else
		Chassis_power.temPower=5500;

	 if(Chassis_power.CapVot<1200)
	 {
		   speedlim=3000; 
		   Control_state.power_state=1;
	 }
	 else if(Control_state.power_state!=0&&Chassis_power.CapVot>1400)
	 {
		   speedlim=7500;
		   Control_state.power_state=0; 
	 }
}

uint16_t temp_power=5500;
void CanTranPower(void)                                    //功率控制
{
	 if(Observer.Tx.DR16_Rate>15)
   {
		 Chassis_power.CanData[0]=(uint8_t)(Chassis_power.temPower>>8);
     Chassis_power.CanData[1]=(uint8_t)(Chassis_power.temPower);
	 }
	 else
   {
		 Chassis_power.CanData[0]=(uint8_t)(temp_power>>8);
     Chassis_power.CanData[1]=(uint8_t)(temp_power);
	 }
	   CAN2_Transmit(0x210,Chassis_power.CanData);
}
static void superCapCanTransmit(void)
{
		int16_t PowerLimit = JUDGE_u16GetChassisPowerLimit();
		if(PowerLimit > 120)
		{
			PowerLimit = 120;
		}
		Chassis.SuperCap.can_data[0] = PowerLimit;
		Chassis.SuperCap.can_data[1] = JUDGE_u16GetRemainEnergy() >> 8;
		Chassis.SuperCap.can_data[2] = JUDGE_u16GetRemainEnergy();
		Chassis.SuperCap.can_data[3] = (uint16_t)JUDGE_f32GetChassisPower() >> 8;
		Chassis.SuperCap.can_data[4] = (uint16_t)JUDGE_f32GetChassisPower();
		Chassis.SuperCap.can_data[5] =	super_state;
		
		CAN1_Transmit(0x210, Chassis.SuperCap.can_data);
}
void Chassis_kb_control(int16_t RxAngle)                      //键盘控制 
{
		RxAngle = Holder.Yaw._0x209.Rx.Angle; 
		for(int i=0;i<4;i++)
		{
			//阻力减速
			if(Control_state.direction[i]==0)
			{
//				Control_state.running_speed[i] = 0;
//				if(Control_state.fly_on.cr_state==0)
//					Control_state.running_speed[i] -= 10;
//				else
					Control_state.running_speed[i] -= 50;
					 
					if(Control_state.running_speed[i]<=0)
				   Control_state.running_speed[i]=0;
			}
      else
			{
					 Control_state.running_speed[i]+=ac;
				  if(Control_state.running_speed[i]>=MAX)
					 Control_state.running_speed[i]=MAX;
		  }
		}

  if(Control_state.spin_move.cr_state==0)
	{
		switch(Control_state.spin_state.cr_state)
	 {
		case 1:				//跟随模式
		   Chassis.MoveData.Front      =        Control_state.running_speed[0]-Control_state.running_speed[1];
		   Chassis.MoveData.Right      =        Control_state.running_speed[3]-Control_state.running_speed[2];
		 if(-5<=Chassis.MoveData.Front&&Chassis.MoveData.Front<=5&&-5<=Chassis.MoveData.Right&&Chassis.MoveData.Right<=5)
		   Chassis.MoveData.ClockWise  =        0x00;
		 else
		 {
			 if(RxAngle > 180)
				Chassis.MoveData.ClockWise  =     -(int16_t)((360-RxAngle)*(360-RxAngle));
			 else
				Chassis.MoveData.ClockWise  =      (int16_t)(RxAngle * RxAngle);
		   if(Chassis.MoveData.ClockWise>=720)
				 Chassis.MoveData.ClockWise=720;
			 if(Chassis.MoveData.ClockWise<=-720)
				 Chassis.MoveData.ClockWise=-720;
		 }
		break;

		case 0:
			 if((RxAngle < 5)||(RxAngle > 355))
				Chassis.MoveData.ClockWise  =   0;//  -(int16_t)((360-RxAngle) *(360-RxAngle) * 0.85);
			 else 
				 if(RxAngle > 180)
				Chassis.MoveData.ClockWise  =     -(int16_t)((360-RxAngle)*(360-RxAngle) * 0.8);
			 else
				Chassis.MoveData.ClockWise  =      (int16_t)(RxAngle * RxAngle * 0.8);
		   if(Chassis.MoveData.ClockWise>=720)
				 Chassis.MoveData.ClockWise=720;
			 if(Chassis.MoveData.ClockWise<=-720)
				 Chassis.MoveData.ClockWise=-720;
		   Chassis.MoveData.Front      =      Control_state.running_speed[0]-Control_state.running_speed[1];
       Chassis.MoveData.Right      =      Control_state.running_speed[3]-Control_state.running_speed[2];
			 
			 int16_t ClockWise_abs       =      (Chassis.MoveData.ClockWise>0)?(Chassis.MoveData.ClockWise):(-Chassis.MoveData.ClockWise);
			 Chassis.MoveData.Front     *=      (1 - ((double)ClockWise_abs/720)/2.2);
		break;
	 }
  }
	else
	{
		//小陀螺模式
     v_front =  Control_state.running_speed[0]-Control_state.running_speed[1];
	   v_right = -Control_state.running_speed[3]+Control_state.running_speed[2];
		 Chassis.MoveData.Front      =   (int16_t)((float)v_front*cos_x(RxAngle) + (float)v_right*sin_x(RxAngle));	
		 Chassis.MoveData.Right      =   (int16_t)((float)v_front*sin_x(RxAngle) - (float)v_right*cos_x(RxAngle)); 
		 int16_t v_front_abs         =   v_front>0?v_front:-v_front;
		 int16_t v_right_abs         =   v_right>0?v_right:-v_right;
		 Chassis.MoveData.ClockWise  =   750-0.3*v_front_abs-0.3*v_right_abs;
	}
}

void Chassis_re_control(int16_t RxAngle,RemoteData_t RDMsg)                     //遥控器控制
{
	RxAngle = Holder.Yaw._0x209.Rx.Angle;                            //将偏航电机角度赋值，另外地方的赋值不知为何无效  
	switch(Control_state.Remote_S2)
	{
		case 1:
		{
//			if(-3<=Chassis.MoveData.Front&&Chassis.MoveData.Front<=3&&-3<=Chassis.MoveData.Right&&Chassis.MoveData.Right<=3)
//				Chassis.MoveData.ClockWise  =        0x00;
			if( (RxAngle>=0&&RxAngle<=5)||(RxAngle>=355&&RxAngle<=360) )
				Chassis.MoveData.ClockWise  =        0x00;
			else 
			if(RxAngle > 180)
				Chassis.MoveData.ClockWise  =     -(int16_t)((360-RxAngle) *(360-RxAngle) * 0.85);
			else
				Chassis.MoveData.ClockWise  =      (int16_t)(RxAngle * RxAngle * 0.85);				
			if(Chassis.MoveData.ClockWise>=720)
				Chassis.MoveData.ClockWise=720;
			if(Chassis.MoveData.ClockWise<=-720)
				Chassis.MoveData.ClockWise=-720;
			Chassis.MoveData.Front      =      0.4*RDMsg.Ch3;
			Chassis.MoveData.Right      =      0.4*RDMsg.Ch2;
			break;
		}
		case 3:
		{
			if( (RxAngle>=0&&RxAngle<=5)||(RxAngle>=355&&RxAngle<=360) )
				Chassis.MoveData.ClockWise  =        0x00;
			else 
			if(RxAngle > 180)
				Chassis.MoveData.ClockWise  =     -(int16_t)((360-RxAngle) *(360-RxAngle) * 0.85);
			else
				Chassis.MoveData.ClockWise  =      (int16_t)(RxAngle * RxAngle * 0.85);				
			if(Chassis.MoveData.ClockWise>=720)
				Chassis.MoveData.ClockWise=720;
			if(Chassis.MoveData.ClockWise<=-720)
				Chassis.MoveData.ClockWise=-720;
			Chassis.MoveData.Front      =      0.4*RDMsg.Ch3;
			Chassis.MoveData.Right      =      0.4*RDMsg.Ch2;
			break;
		}
		case 2:                                                     //小陀螺模式改进
		{					
			v_front =  RDMsg.Ch3;
			v_right =  RDMsg.Ch2;
			Chassis.MoveData.Front      =   (int16_t)((float)v_front*cos_x(RxAngle) - (float)v_right*sin_x(RxAngle));
			Chassis.MoveData.Right      =   (int16_t)((float)v_front*sin_x(RxAngle) + (float)v_right*cos_x(RxAngle));
			int16_t v_front_abs         =   v_front>0?v_front:-v_front;
			int16_t v_right_abs         =   v_right>0?v_right:-v_right;
			Chassis.MoveData.ClockWise  =   750-0.3*v_front_abs-0.3*v_right_abs;
			break;
		}
	}
}

void Chassis_Process(RemoteData_t RDMsg,HolderData_t HDMsg)           //处理过程
{
		Chassis_GetMoveData(RDMsg,HDMsg);
		Chassis_CanTransmit();
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
