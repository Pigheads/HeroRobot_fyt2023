/**
  ******************************************************************************
  * @file    
  * @author  sy，lj
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
#include "can.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define Speed_Fix 1100

/* variables -----------------------------------------------------------------*/
Ammunition_t Ammunition;
uint8_t temp_block=0;
uint8_t state=0;
uint8_t temp_reverse=0;
uint8_t temp_empty=0;
uint8_t flag=0;
/* function ------------------------------------------------------------------*/
double AmP=20,AmI=0,AmD=0;
void Ammunition_PidInit(void)
{
//     pid_init_antiintegral(&Ammunition.PidSpeed_A,6.7,0.13,4.6,13000,4000,0);
//	   pid_init_antiintegral(&Ammunition.PidAngle_A,0.62,0.0,0.0,3000,0,0);
	pid_init_increment(&Ammunition.PidSpeed,3,2.55,2,5000,10000);		//15.3,0.45,2.0
	pid_init_increment(&Ammunition.PidCurrent,1.48,0.205,0.020,5000,10000);
//		 pid_init_increment(&Ammunition.PidSpeed,AmP,AmI,AmD,5000,10000);
//	   pid_init_increment(&Ammunition.PidCurrent,1.60,0.205,0.005,5000,10000);
	flag=0;
	Ammunition.TarSpeed = 0;
	Ammunition.TarCurrent = 0;
}
int testHP;
int Heat_Prediction(uint16_t Q1,uint16_t Q0,uint16_t Qs)
{	 
	int16_t Max_HP=JUDGE_u16GetMaxHP();
	testHP = Max_HP;
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
int H=0;
void HeatControl(void)
{
	uint16_t Q1=JUDGE_u16GetRemoteHeat42();
	uint16_t Q0=JUDGE_u16GetHeatLimit();
	uint16_t Qs=JUDGE_u16GetHeatRate();
	H = Q1;
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

void Shootblock_detection(void)
{
	if(Ammunition._0x206.OutputLpf>=9500)
		temp_block++;
	else
		temp_block=0;
	if(temp_block>=95&&(-10<Ammunition.RxSpeed)&&(Ammunition.RxSpeed<10))
	{
		temp_block=0;
		Control_state.ShootBlock=1;
	}

}

void Ammunition_protect(void)
{
	Ammunition.TarAngle = Ammunition.RxAngleLPf;
	Ammunition.TarSpeed = 0;
	Ammunition.PidSpeed.dCtrOut = 0;
	Ammunition.PidSpeed.ctrOut = 0;
	Ammunition._0x206.Output = 0;
	Ammunition._0x206.OutputLpf = 0;
	flag = 0;
}

void ShootEmpty_detection(void)
{
	if(Ammunition.RxSpeed>=1000)
		temp_empty++;
	if(temp_empty>=200)
	{
		temp_empty=0;
		flag=0;
		Ammunition.ShootFlag[0]=flag;
		CAN2_Transmit(0x300,Ammunition.ShootFlag);
		Ammunition_protect();
	}
}

void Ammunition_kb_control(void)
{
//   if((Control_state.HeatPermit==0)&&(Control_state.ShootBlock==0))
	if(Control_state.ShootBlock==0)
	 {
			if(Control_state.Remote_S1!=2)	//键鼠控制 鼠标左键
			{
			 Ammunition_State(Control_state.ammunition);
			 if(flag==1)
				 Ammunition.TarSpeed = Speed_Fix;
			 else
				 Ammunition_protect();
			}
			else 
			{																//遥控器
			 Ammunition_State(Control_state.Remote_Wheel>=330||Control_state.Remote_Wheel<=-330);
			 if(flag==1)
				 Ammunition.TarSpeed = Speed_Fix;
			 else
				 Ammunition_protect();
			}
	 }
	 else if(Control_state.ShootBlock==1)
	 {
		 temp_reverse++;
		 if(temp_reverse<=70)
		 {
			 Ammunition.TarSpeed = -Speed_Fix;
		 }
		 else
		 {
				temp_reverse=0;
				flag=0;
				Control_state.ShootBlock=0;
				Ammunition.ShootFlag[0]=flag;
				CAN2_Transmit(0x300,Ammunition.ShootFlag);
		 }
	 }
	 else
	 {
			 flag=0;
			 Ammunition.ShootFlag[0]=flag;
			 CAN2_Transmit(0x300,Ammunition.ShootFlag);
       Ammunition_protect();
	 }
}

int AmmunitionRx;
void Ammunition_GetMoveData(void)
{
	if(Observer.Tx.DR16_Rate>15)			
	{	
		Ammunition_kb_control();
	}
	else	
	{
		Ammunition.TarAngle = Ammunition.RxAngle;;
		Ammunition.TarSpeed = 0;
		Ammunition.PidSpeed.dCtrOut = 0;
		Ammunition.PidSpeed.ctrOut = 0;
		Ammunition._0x206.Output = 0;
		Ammunition._0x206.OutputLpf = 0;
		Control_state.SingleShoot = 0;
		flag=0;
	}
}

void Ammunition_LPfIn(void)
{
	AmmunitionRx = Ammunition.RxSpeed;
	Ammunition.RxSpeedLPf  = 0.8*Ammunition.RxSpeed   +0.2*Ammunition.RxSpeedLPf;
	Ammunition.RxCurrentLPf= 0.8*Ammunition.RxCurrent +0.2*Ammunition.RxCurrentLPf;
}

void Ammunition_LPfOut(void)
{
	Ammunition._0x206.OutputLpf=0.75*Ammunition._0x206.Output+0.25*Ammunition._0x206.OutputLpf;
}

void Ammunition_PidRun(void)
{
//			   Ammunition.TarSpeed      = pid_antiintegral_update(Ammunition.TarAngle,Ammunition.RxAngleLPf,&Ammunition.PidAngle_A);
//         Ammunition._0x206.Output = pid_antiintegral_update(Ammunition.TarSpeed,Ammunition.RxSpeedLPf,&Ammunition.PidSpeed_A);

		Ammunition.TarCurrent    = pid_increment_update(Ammunition.TarSpeed,Ammunition.RxSpeedLPf,&Ammunition.PidSpeed);
		Ammunition._0x206.Output = pid_increment_update(Ammunition.TarCurrent,Ammunition.RxCurrentLPf,&Ammunition.PidCurrent);

//					Ammunition._0x206.Output = pid_increment_update(Ammunition.TarSpeed,Ammunition.RxSpeedLPf,&Ammunition.PidSpeed);
}

void Ammunition_CanTransmit(void)
{
    if(Observer.Tx.DR16_Rate>15)				
    {
				Ammunition.CanData2[2]=(uint8_t)(Ammunition._0x206.OutputLpf>>8);
				Ammunition.CanData2[3]=(uint8_t)(Ammunition._0x206.OutputLpf);
		}
    else
    {
				memset(Ammunition.CanData2,0,sizeof(Ammunition.CanData2));
				memset(Ammunition.ShootFlag,0,sizeof(Ammunition.ShootFlag));
		}

}

void Ammunition_Process(void)
{
	  HeatControl();						//热量控制
	  Shootblock_detection();		//卡弹保护 
	  ShootEmpty_detection();		//打空保护 
	  Ammunition_LPfIn();
    Ammunition_GetMoveData();
		if(Observer.Tx.DR16_Rate>15&&(Control_state.HeatPermit==0)&&flag==1)
		{
			Ammunition_PidRun();
		}
		Ammunition_LPfOut();
    Ammunition_CanTransmit();
}


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
