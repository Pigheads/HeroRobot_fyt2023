/**
  ******************************************************************************
  * @file    
  * @author  ycz,sy,lj
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
  * License. You may obtain a copy of the License at:yu'i
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
	
/*
  1.云台跟随模式停止时会偏移
  2.底盘不硬，拨弹力太大时会影响底盘（待测试）
  3.瞄准模式左右移动不均
	
	改进方向：
	1.r键瞄准模式开启瞬间记录当前云台角度作为目标值，改为另一套PID控制，锁定当前角度射击，要做到射击时方向不偏，主要打前哨站和基地。
	2.尝试解决跟随模式下云台莫名偏移的问题。
	3.吊射测试，画UI，要有射表。
	4.飞坡功能编写和测试（根据陀螺仪偏角加速）。
	5.在后续测试中时刻注意有无卡弹，连发问题，随时改进。
	6.自瞄的改进，跟踪稳定性和可控性。
	
*/

/* includes ------------------------------------------------------------------*/
#include "holder.h"
#include "motor.h"
#include "can.h"
#include <string.h>
#include <tim.h>
#include "keyboard.h"
#include "vision.h"
#include "mathfun.h"
#include "judge_rx.h"
/* typedef -------------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
 
/* variables -----------------------------------------------------------------*/
Holder_t Holder={0};int16_t pitch_y=0;
int16_t pitch_x=0;
int16_t Aim_Angle_Yaw=0;
int16_t Aim_Angle_Pitch=0;

float temp_kp_p[3]={0.1,0.09,0.08};//俯仰三段式
float temp_err_p[3]={50,150,300};

float temp_kp_y[4]={0.76,0.9,1.0};//偏航三段式
float temp_err_y[4]={20,35,90};
/* function -----------------------------------------------------------------*/
//int debuger_kp=350,debuger_ki=64,debuger_kd=77;
//int debuger_kp=617,debuger_ki=63,debuger_kd=1750;
int debuger_kp=627,debuger_ki=88,debuger_kd=2000;

void Holder_PidInit(void)
{
//   	pid_init_antiintegral(&Holder.Yaw.PidSpeed_Y,1396,68,1950,30000,20000,25);//1426,185,1950,20000,6
//	  pid_init_antiintegral(&Holder.Yaw.PidAngle_Y,0.9,0,0,30000,0,0);//6.5,0,0.2

    
//    pid_init_antiintegral(&Holder.Yaw.PidSpeed,1386,60,1985,30000,20000,20);//1368,55,1980,30000,20000,20	
//	  pid_init_absolute_threesection(&Holder.Yaw.PIDAngle,temp_kp_y,temp_err_y,0,0.1,0,30000);//kp[3],err[3],ki,kd,I_Limit,Out_Limit	
//		pid_init_antiintegral(&Holder.Yaw.PidSpeed,1350,35,1985,30000,20000,20);//1368,55,1980,30000,20000,20
	  pid_init_absolute_threesection(&Holder.Yaw.PIDAngle,temp_kp_y,temp_err_y,0.01,0,95,30000);//kp[3],err[3],ki,kd,I_Limit,Out_Limit	
		pid_init_antiintegral(&Holder.Yaw.PidSpeed,debuger_kp,debuger_ki,debuger_kd,30000,20000,20);//1368,55,1980,30000,20000,20
	
	  //Holder.Yaw._0x209.TarAngle = Control_state.gyoyaw_angle;
}

void Holder_Protect(HolderData_t HDMsg)
{
		 Holder.Yaw._0x209.TarAngle = Holder.Yaw._0x209.Rx.Angle;
		 Holder.Yaw._0x209.TarSpeed = Holder.Yaw._0x209.Rx.Speed;
		 Holder.Yaw._0x209.Output=0;
		 Holder.Yaw._0x209.OutputLpf=0;
	   Holder.Yaw.PidSpeed.errNow=0;
	   Holder.Yaw.PidSpeed.errOld=0;
	   Holder.Yaw.PidSpeed.errP=0;
	   Holder.Yaw.PidSpeed.errD=0;

}

void Holder_LPfIn(void)
{
	  Holder.Yaw._0x209.TarAngleLpf   = Receive_filter_4(Holder.Yaw._0x209.TarAngle,Holder.Yaw.Filter_Rx_Buf);
}

void Holder_re_control(RemoteData_t RDMsg)
{
     Holder.Yaw._0x209.TarAngle  -= 0.015*RDMsg.Ch0;
		 pitch_y -=0.015*RDMsg.Ch1;
		 Holder.Pitch._0x20A.TarAngle = pitch_y+4675; 
}

void Holder_kb_control(RemoteData_t RDMsg)
{
		 if(Control_state.vision==1&&vData.Pos.z!=-1)
			{
				   Holder.Yaw._0x209.TarAngle    = Holder.Yaw.Rx.Angle + 2*vData.Pos.x;
					 pitch_y +=0.35*RDMsg.Mouse_y+5.0*(RDMsg.KeyBoard.c)-5.0*(RDMsg.KeyBoard.z);
			     Holder.Pitch._0x20A.TarAngle  = pitch_y+4675;
				   Control_state.vision_state=1;
      }
		 else
			{
				 Control_state.vision_state=0;
				 if(Control_state.aim_state.cr_state==1)
				 {
			     Holder.Yaw._0x209.TarAngle   = Aim_Angle_Yaw+Control_state.yaw_angle;
				   Holder.Pitch._0x20A.TarAngle = Aim_Angle_Pitch+Control_state.pitch_angle;
				 }
				 else
				 {
					 Aim_Angle_Yaw   = Holder.Yaw.Rx.Angle;
					 Aim_Angle_Pitch = Holder.Pitch._0x20A.Rx.Angle;
					 Control_state.yaw_angle=0;
					 Control_state.pitch_angle=0;
					 Holder.Yaw._0x209.TarAngle   -= 0.5*RDMsg.Mouse_x;
			     Holder.Yaw._0x209.TarAngle   += 0.03*(RDMsg.KeyBoard.q)-0.03*(RDMsg.KeyBoard.e);
			     pitch_y +=0.5*RDMsg.Mouse_y+4.0*(RDMsg.KeyBoard.c)-4.0*(RDMsg.KeyBoard.z);
			     Holder.Pitch._0x20A.TarAngle  = pitch_y+4675;
				 }
			}
}

void Holder_GetMoveData(RemoteData_t RDMsg,HolderData_t HDMsg)
{
//	   Holder.Yaw.Rx.Angle   = HDMsg.Angle;
//		 Holder.Yaw.Rx.Speed   = HDMsg.Speed;
//	   Holder.Pitch.gyoSpeed = HDMsg.PxSpeed;
//	   Holder.Pitch.gyoAngle = HDMsg.PxAngle;
		 if(Observer.Tx.DR16_Rate>15)
		 {
//      if(Control_state.Remote_S1!=2)
//			 Holder_kb_control(RDMsg);
//			else
//			 Holder_re_control(RDMsg);
//			
			 if(Holder.Pitch._0x20A.TarAngle>=5000)
				{
			     Holder.Pitch._0x20A.TarAngle=5000;
					 pitch_y = 325;
				}
		   if(Holder.Pitch._0x20A.TarAngle<=3900)
				{
			     Holder.Pitch._0x20A.TarAngle=3900;
					 pitch_y = -775;
				}
		 }
		 else
		 {	 
       Holder_Protect(HDMsg);
		 }
}
int TarAngle=0;
int RxAngle=0;
void Holder_PidRun(void)
{
	static uint8_t ticks = 0;
	ticks++;
//      Holder_Pi_Tunning(3,4.2,0,0,&Holder.Yaw.PidAngle); //变参数PID 云台偏航
	if(ticks == 5)	//位置环控制10ms，速度环2ms
	{
		ticks = 0;
		Holder.Yaw._0x209.TarSpeed=(int16_t)pid_absolute_threesection_update(Holder.Yaw._0x209.TarAngleLpf,Holder.Yaw.Rx.Angle,&Holder.Yaw.PIDAngle); //三段绝对式角度环
	}
	Holder.Yaw._0x209.Output=(int16_t)pid_antiintegral_update(Holder.Yaw._0x209.TarSpeed,Holder.Yaw.Rx.Speed,&Holder.Yaw.PidSpeed);
	TarAngle = Holder.Yaw._0x209.TarAngle;
	RxAngle = Holder.Yaw.Rx.Angle;
//      Holder.Yaw._0x209.TarSpeed=(int16_t)pid_antiintegral_update(Holder.Yaw._0x209.TarAngleLpf,Holder.Yaw.Rx.Angle,&Holder.Yaw.PidAngle); //抗积分饱和绝对式角度环
//  	  Holder.Yaw.Output=(int16_t)pid_antiintegral_update(Holder.Yaw._0x209.TarSpeed,Holder.Yaw.Rx.Speed,&Holder.Yaw.PidSpeed);      

}

void Holder_CanTransmit(void)
{
	if(Observer.Tx.DR16_Rate>15)
	{
	  for(int i=0;i<4;i++)
	   {
			   Holder.Yaw.CanData[2*i]   = (uint8_t)(Holder.Yaw._0x209.OutputLpf>>8);
			   Holder.Yaw.CanData[2*i+1] = (uint8_t) Holder.Yaw._0x209.OutputLpf;			 
		 }
	}
	else
	{
		 memset(Holder.Yaw.CanData,0,sizeof(Holder.Yaw.CanData));
	}
//	CAN2_Transmit(0x400,YawData);
}

void Holder_LPfOut(void)
{
      Holder.Yaw ._0x209.OutputLpf = 0.68*Holder.Yaw._0x209.Output+0.32*Holder.Yaw._0x209.OutputLpf;
}

void HolderMsg_Process(RemoteData_t RDMsg,HolderData_t HDMsg)
{
	//Holder_LPfOut();
  Holder_CanTransmit();
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
