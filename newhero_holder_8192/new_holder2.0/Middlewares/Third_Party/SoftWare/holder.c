/**
  ******************************************************************************
  * @file    
  * @author  sy,lj
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
 #define zhongjianjiaodupianyi 1120
/* variables -----------------------------------------------------------------*/
Holder_t Holder;
int16_t pitch_y=0;
int16_t pitch_x=0;
int16_t Aim_Angle_Yaw=0;
int16_t Aim_Angle_Pitch=0;
float temp_kp_p[4]={0.26,0.26,0.26,0.27};//俯仰三段式0.35,0.35,0.36,0.37  0.1,0.09,0.08,0.08 0.09,0.08,0.06,0.06  0.09,0.08,0.1,0.1
float temp_err_p[4]={10,50,100,270};
float temp_kp_y[4]={0.72,0.74,0.76,0.74};//偏航三段式 2,2,2.4,2.5//0.65,0.75,0.75,0.6//0.7,0.7,0.8,0.7
float temp_err_y[4]={5,10,23,90};							//0,7,23,90
float debuger_kp=190,debuger_ki=1.65,debuger_kd=135;
/* function ------------------------------------------------------------------*/
uint8_t Data[8] = {0};
int16_t TarAngle;
int16_t Output;
int16_t RxAngle;
void Holder_PidInit(void)
{
//pid_init_increment(&Holder.Pitch.PIDSpeed,715,109,667,30000,30000);//648,35,160,30000,30000 758,95,690,30000,30000
//pid_init_antiintegral(&Holder.Yaw.PidSpeed,debuger_kp,debuger_ki,debuger_kd,30000,20000,20);//1368,55,1980,30000,20000,20
//	pid_init_absolute(&Holder.Yaw.PidSpeed, 25.0, 0.0, 1, 9999);		//errlim不为0 易导致稳态误差
	pid_init_absolute_threesection(&Holder.Yaw.PIDAngle,temp_kp_y,temp_err_y,0.001,0.01,4,30000);//kp[3],err[3],ki,kd,I_Limit,Out_Limit	
	pid_init_antiintegral(&Holder.Yaw.PidSpeed,98,0.57,989,30000,25000,20);//944,33.7,1160,21000,25000,15//340,0.4,1100,30000,30000,20//1368,55,1980,30000,20000,20	
	
	
	// 俯仰初始化 
	pid_init_absolute_threesection(&Holder.Pitch.PIDAngle,temp_kp_p,temp_err_p,0,0.08,0,30000);//0,0.001,2,30000
//	pid_init_increment(&Holder.Pitch.PIDSpeed,688,0.5,668,30000,30000);
	pid_init_absolute(&Holder.Pitch.PIDSPEED,debuger_kp,debuger_ki,debuger_kd,30000);
//	pid_init_antiintegral(&Holder.Pitch.PidSpeed,167,8.6,112,30000,30000,20);
}
int PitchTarSpeed;
int PitchTarAngle;
int PitchRxSpeed;
int PitchRxAngle;
int PitchOutput;
int YawTarSpeed;
int YawRxSpeed;
int YawTarAngle;
int YawRxAngle;
void Holder_PidRun(void)
{
//	static uint8_t ticks = 0;
//	ticks++;
//	//偏航
//	
//	if(ticks == 5)
//	{
//		ticks=0;
		Holder.Yaw._0x209.TarSpeed=(int16_t)pid_absolute_threesection_update(Holder.Yaw._0x209.TarAngleLpf,Holder.Yaw.Rx.Angle,&Holder.Yaw.PIDAngle); //三段绝对式角度环	
//	}
	Holder.Yaw._0x209.Output=(int16_t)pid_antiintegral_update(Holder.Yaw._0x209.TarSpeed,Holder.Yaw.Rx.Speed,&Holder.Yaw.PidSpeed);	

//	Holder.Yaw._0x209.Output=(int16_t)pid_absolute(Holder.Yaw._0x209.TarSpeed,Holder.Yaw.Rx.Speed,&Holder.Yaw.PidSpeed);	
	YawTarSpeed = Holder.Yaw._0x209.TarSpeed;
	YawRxSpeed = Holder.Yaw.Rx.Speed;
	YawTarAngle = Holder.Yaw._0x209.TarAngleLpf;
	YawRxAngle = Holder.Yaw.Rx.Angle;
	//俯仰
//	if(ticks == 5)
//	{
//		ticks=0;
	Holder.Pitch._0x20A.TarSpeed=(int16_t)pid_absolute_threesection_update(Holder.Pitch._0x20A.TarAngleLpf,Holder.Pitch._0x20A.Rx.AngleLpf,&Holder.Pitch.PIDAngle);
//	}
//	Holder.Pitch._0x20A.Output=(int16_t)pid_antiintegral_update(Holder.Pitch._0x20A.TarSpeed,Holder.Pitch._0x20A.Rx.Speed,&Holder.Pitch.PidSpeed);
//	Holder.Pitch._0x20A.Output=(int16_t)pid_increment_update(Holder.Pitch._0x20A.TarSpeed,Holder.Pitch.gyoSpeed,&Holder.Pitch.PIDSpeed);
	Holder.Pitch._0x20A.Output=(int16_t)pid_absolute(Holder.Pitch._0x20A.TarSpeed,Holder.Pitch._0x20A.Rx.Speed,&Holder.Pitch.PIDSPEED);
	PitchTarSpeed = Holder.Pitch._0x20A.TarSpeed;
	PitchRxSpeed = Holder.Pitch._0x20A.Rx.Speed;
	PitchTarAngle = Holder.Pitch._0x20A.TarAngleLpf;
	PitchRxAngle = Holder.Pitch._0x20A.Rx.AngleLpf;
	PitchOutput = Holder.Pitch._0x20A.Output;
}

void Holder_Protect(HolderData_t HDMsg)
{
		 Holder.Yaw._0x209.TarAngle = HDMsg.Angle;
		 Holder.Yaw._0x209.TarSpeed = 0;
		 Holder.Yaw._0x209.Output=0;
		 Holder.Yaw._0x209.OutputLpf=0;
	   Holder.Yaw.PidSpeed.errNow=0;
	   Holder.Yaw.PidSpeed.errOld=0;
	   Holder.Yaw.PidSpeed.errP=0;
	   Holder.Yaw.PidSpeed.errD=0;
	   Holder.Pitch._0x20A.TarAngle = Holder.Pitch._0x20A.Rx.Angle;
	   Holder.Pitch._0x20A.TarSpeed = 0;
	   pitch_y=0;
	   Holder.Pitch._0x20A.Output=0;
		 Holder.Pitch._0x20A.OutputLpf=0;
	   Holder.Pitch.PidSpeed.errNow=0;
	   Holder.Pitch.PidSpeed.errOld=0;
	   Holder.Pitch.PidSpeed.errP=0;
		Holder.Pitch.PidSpeed.errI=0;
	   Holder.Pitch.PidSpeed.errD=0;
}

void Holder_LPfIn(void)
{
	  Holder.Yaw._0x209.TarAngleLpf   = Receive_filter_4(Holder.Yaw._0x209.TarAngle,Holder.Yaw.Filter_Rx_Buf);
//		Holder.Yaw.Rx.AngleLpf 						= Holder.Yaw.Rx.Angle;//Yaw_Rx_Lpf(Holder.Yaw.Rx.Angle);
		Holder.Pitch._0x20A.TarAngleLpf = Receive_filter_4(Holder.Pitch._0x20A.TarAngle,Holder.Pitch.Filter_Rx_Buf);
	  Holder.Pitch._0x20A.Rx.AngleLpf = Pitch_Rx_Lpf(Holder.Pitch._0x20A.Rx.Angle);
}
int vision_y;
void Holder_re_control(RemoteData_t RDMsg)
{
	/* 视觉控制算法 */
//	Holder.Yaw._0x209.TarAngle    = Holder.Yaw.Rx.Angle + 1.0*vData.Pos.y;
//	if(vData.Pos.y <= 5 && vData.Pos.y >= -5)
//		Holder.Yaw._0x209.TarAngle    = Holder.Yaw._0x209.TarAngle + 0.2*vData.Pos.y;
//	else 
//		Holder.Yaw._0x209.TarAngle    = Holder.Yaw._0x209.TarAngle + 1*vData.Pos.y;
////	vision_y = vData.Pos.y;
//	pitch_y += 0.5*vData.Pos.x;	
//	
	/* 遥控器控制算法 */

    Holder.Yaw._0x209.TarAngle  -= 0.010*RDMsg.Ch0;
		pitch_y +=0.010*RDMsg.Ch1;
	
		Holder.Pitch._0x20A.TarAngle = pitch_y+6000;
}
int Q;
void Holder_kb_control(RemoteData_t RDMsg)
{
		 if(Control_state.vision==1&&vData.Pos.z!=-1)
			{
				  Holder.Yaw._0x209.TarAngle    = Holder.Yaw._0x209.TarAngle + 1.0*vData.Pos.y;
				
//					pitch_y -= 0.5*RDMsg.Mouse_y+4.0*(RDMsg.KeyBoard.c)-4.0*(RDMsg.KeyBoard.z)-0.5*vData.Pos.z;
//					pitch_y += -0.5*RDMsg.Mouse_y+4.0*(RDMsg.KeyBoard.c)-4.0*(RDMsg.KeyBoard.z);
				pitch_y += 0.5*vData.Pos.x;
					Holder.Pitch._0x20A.TarAngle  = pitch_y+6000;
				
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
					 if(RDMsg.Mouse_x>=-10 && RDMsg.Mouse_x<=10)
							Holder.Yaw._0x209.TarAngle   -= 0.5*RDMsg.Mouse_x;
					 else
						 Holder.Yaw._0x209.TarAngle   -= 1*RDMsg.Mouse_x;
			     Holder.Yaw._0x209.TarAngle   += 1.0*(RDMsg.KeyBoard.q)-1.0*(RDMsg.KeyBoard.e);
					 pitch_y -= 0.5*RDMsg.Mouse_y+2.0*(RDMsg.KeyBoard.c)-2.0*(RDMsg.KeyBoard.z);
					 Q = RDMsg.KeyBoard.q;
					 Holder.Pitch._0x20A.TarAngle  = pitch_y+6000;				 
				 }
			}
}


void Holder_GetMoveData(RemoteData_t RDMsg,HolderData_t HDMsg)
{
		 Holder.Yaw.Rx.Speed   = HDMsg.Speed;
	   Holder.Yaw.Rx.Angle   = HDMsg.Angle;
	
	   Holder.Pitch.gyoSpeed = HDMsg.PxSpeed;
	   Holder.Pitch.gyoAngle = HDMsg.PxAngle;
	
//kalman滤波预测
//	 Holder_CoordinateTransform();
//	 vision_predict();
	
		 if(Observer.Tx.DR16_Rate>15)
		 {
      if(Control_state.Remote_S1!=2)
			 Holder_kb_control(RDMsg);
			else
			 Holder_re_control(RDMsg);
			
			 if(Holder.Pitch._0x20A.TarAngle>=6550)
				{
			     Holder.Pitch._0x20A.TarAngle=6550;
					 pitch_y = 6550-6000;
				}	
			 if(Holder.Pitch._0x20A.TarAngle<=5670)
				{
			     Holder.Pitch._0x20A.TarAngle=5670;
					 pitch_y = 5670-6000;
				}
		 
		}
		 else
		 {	 
			 Holder_Protect(HDMsg);
		 }
}
void Holder_CanTransmit(void)
{
	if(Observer.Tx.DR16_Rate>15)
	{
		 Holder.Yaw.CanData[0]   =   (uint8_t)(Holder.Yaw ._0x209.OutputLpf>>8);         //云台解算的偏航输出值转发给底盘
		 Holder.Yaw.CanData[1]   =   (uint8_t)(Holder.Yaw ._0x209.OutputLpf);
		 Holder.Yaw.CanData[6]   =   (uint8_t)(Control_state.Remote_Wheel>>8)	;	         //遥控器滑轮  
		 Holder.Yaw.CanData[7]   =   (uint8_t)(Control_state.Remote_Wheel)	;
		 Holder.Pitch.CanData[0] =   (uint8_t)((Holder.Pitch._0x20A.OutputLpf)>>8);
		 Holder.Pitch.CanData[1] =   (uint8_t)(Holder.Pitch._0x20A.OutputLpf);
		 Holder.Pitch.CanData[2] =   (uint8_t)((Holder.Pitch._0x20A.OutputLpf)>>8);
		 Holder.Pitch.CanData[3] =   (uint8_t)(Holder.Pitch._0x20A.OutputLpf);
	}
	else
	{
		 memset(Holder.Yaw.CanData,0,sizeof(Holder.Yaw.CanData));
		 memset(Holder.Pitch.CanData,0,sizeof(Holder.Pitch.CanData));
	}
}

void Holder_LPfOut(void)
{
	    Holder.Pitch ._0x20A.OutputLpf = 0.7*Holder.Pitch._0x20A.Output+0.3*Holder.Pitch._0x20A.OutputLpf;
      Holder.Yaw ._0x209.OutputLpf = 0.68*Holder.Yaw._0x209.Output+0.32*Holder.Yaw._0x209.OutputLpf;
}

void HolderMsg_Process(RemoteData_t RDMsg,HolderData_t HDMsg)
{
	Holder_GetMoveData(RDMsg,HDMsg);
	Holder_LPfIn();
	if(Observer.Tx.DR16_Rate>15)
	{
			Holder_PidRun();
	}
	Holder_LPfOut();
  Holder_CanTransmit();
}



/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
