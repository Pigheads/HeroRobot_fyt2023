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
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "holder.h"
#include <string.h>
#include "tim.h"
#include "can.h"
#include "keyboard.h"
#include "judge_rx.h"
#include "mathfun.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
#define CHASSIS_NORMAL    1
#define CHASSIS_SUPERCAP  0
#define MAX 500
#define MAX_spin 300
#define MAX_su 600
#define ac 4
#define ac_shfit 12
/* variables -----------------------------------------------------------------*/
Chassis_t Chassis={0};
PowerData_t Chassis_power = {0};

int16_t  v_front=0;
int16_t  v_right=0;

uint16_t speedlim = 8000; //7500;
uint8_t super_state    =  0;
int16_t fmax_wheel_speed = 8000;/*< ����ƶ��ٶ� */
int16_t fmax_spin_speed = 720; /*< �����ת�ٶ� */
int16_t fmax_spin_static = 800;/*<��󶨵���ת�ٶ�*/
float acc_max = 2.5f;
/* function ------------------------------------------------------------------*/
double pp1=7.15,ii1=0.48,dd1=6.5;
double pp2=3.7,ii2=0.3,dd2=0.05;
void Chassis_PidInit(void)
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
//        pid_init_increment(&Chassis.M3508[i].PidSpeed,pp1,ii1,dd1,16000,16000);
				pid_init_increment(&Chassis.M3508[i].PidSpeed,7.25,0.38,6.5,16000,16000);
    }
    for(i=0;i<4;i++)
    {
//        pid_init_increment(&Chassis.M3508[i].PidCurrent,pp2,ii2,dd2,16000,16000);
				pid_init_increment(&Chassis.M3508[i].PidCurrent,0.9,0.2,0.05,16000,16000);
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
		int16_t RxAngle = Holder.Yaw._0x209.Rx.Angle; 	
		int16_t ramp_Coeff = 5;
		float maxspeed = speedlim;
//		if(super_state == CHASSIS_SUPERCAP)
//			ramp_Coeff += 3;
	  if(Observer.Tx.DR16_Rate>15)
		{
			//б�»���
			Chassis.RampMoveData.Front = Ramp_function(66, Chassis.MoveData.Front, Chassis.RampMoveData.Front, ramp_Coeff);
			Chassis.RampMoveData.Right = Ramp_function(66, Chassis.MoveData.Right, Chassis.RampMoveData.Right, ramp_Coeff);
//      Chassis.RampMoveData.Front = Chassis.MoveData.Front;
//			Chassis.RampMoveData.Right = Chassis.MoveData.Right;
			Chassis.RampMoveData.ClockWise = Chassis.MoveData.ClockWise;
			//�˶�ѧ����
			Chassis.M3508[0].TarSpeed = (int16_t)(  Chassis.RampMoveData.Right / GAIN_I + Chassis.RampMoveData.Front / GAIN_J + Chassis.RampMoveData.ClockWise / GAIN_K);
      Chassis.M3508[1].TarSpeed = (int16_t)(  Chassis.RampMoveData.Right / GAIN_I - Chassis.RampMoveData.Front / GAIN_J + Chassis.RampMoveData.ClockWise / GAIN_K);
      Chassis.M3508[2].TarSpeed = (int16_t)( -Chassis.RampMoveData.Right / GAIN_I + Chassis.RampMoveData.Front / GAIN_J + Chassis.RampMoveData.ClockWise / GAIN_K);
      Chassis.M3508[3].TarSpeed = (int16_t)( -Chassis.RampMoveData.Right / GAIN_I - Chassis.RampMoveData.Front / GAIN_J + Chassis.RampMoveData.ClockWise / GAIN_K);
			
			//�����ٶȰ������޷�
			for(int i=0;i<4;i++)
			{
				if(abs(Chassis.M3508[i].TarSpeed) > maxspeed)
					maxspeed = abs(Chassis.M3508[i].TarSpeed);
			}
			if(maxspeed != speedlim)
			{
				for(int i=0;i<4;i++)
					Chassis.M3508[i].TarSpeed = (float)Chassis.M3508[i].TarSpeed * ((float)speedlim / maxspeed);
			}

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
			//PID������������޷�
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

void Chassis_CanTransmit(void)
{
    uint8_t i;
    if(Observer.Tx.DR16_Rate>15)
    {
        for(i=0;i<4;i++)
        {
            Chassis.CanData[2*i]=(uint8_t)(Chassis.M3508[i].OutputLpf>>8);
            Chassis.CanData[2*i+1]=(uint8_t)(Chassis.M3508[i].OutputLpf);
        }
    }   
    else
    {
        memset(Chassis.CanData,0,sizeof(Chassis.CanData));
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

static void superCapCanTransmit(void)
{
		float power_limit_tx = JUDGE_u16GetChassisPowerLimit()-5;
		 Chassis.SuperCap.can_data[0] = (uint8_t)power_limit_tx-10;
		 Chassis.SuperCap.can_data[1] = super_state;
		 Chassis.SuperCap.can_data[2] = Chassis.SuperCap.can_data[0] + 10;
		 CAN1_Transmit(0x210, Chassis.SuperCap.can_data);
}

/**
  * @brief  ȡ����ֵ
  * @param  num
  * @retval |num|
  * @attention  
  */
int16_t absFloat(int16_t num)
{
    int16_t abs_num;
    abs_num = num>=0? num:-num;
    return abs_num;
}

void Chassis_PowerControl(void)
{
	//�������ݿ�������
	static uint8_t tick = 0;
	
	//���ݻ��������޷�
	int16_t remain_energy = JUDGE_u16GetRemainEnergy();//��������
	uint8_t RemainParam = 7;
	float P=1.0;
	int32_t sum_output = 0;
	
//	tick ++;
//	if(tick == 20)
//	{
		//���ݱ��� ֻҪ���ݵĵ���ֵ����ĳֵ�����̹رյ���
//		if(Control_state.open_cap.cr_state == 1)
//		{
//			if(super_state == CHASSIS_NORMAL)
//				super_state = CHASSIS_SUPERCAP;
//		}else if(Control_state.open_cap.cr_state == 0)
//		{
//			if(super_state == CHASSIS_SUPERCAP)
//				super_state = CHASSIS_NORMAL;
//		}
//		
		if(Chassis.SuperCap.cap_vol <= 12000 && super_state == CHASSIS_SUPERCAP)
		{
			super_state = CHASSIS_NORMAL;
		}
		//�տ��� �� �������ݵ���ֵ�ָ���ĳֵ�����¿�������
		if(Chassis.SuperCap.cap_vol >= 15000 && super_state == CHASSIS_NORMAL)
		{
			super_state = CHASSIS_SUPERCAP;
		}
		
		//���ͳ������ݿ�������
		superCapCanTransmit();
//		tick = 0;
//	}
	/*�󽻹��ʿ���
	���ݲ���ϵͳ������60j�������� ���Ƶ�����Output
	��ʣ�໺������С��50jʱ��������
	�����ĸ�����������֮��<=ʣ��������ƽ��*ϵ��P
	*/
	if(remain_energy < 40)
	{
		for(int i=0;i<4;i++)
			sum_output += abs(Chassis.M3508[i].Output);
		/*���ĸ�����������֮��>ʣ��������ƽ��*ϵ��
		�ȱ��������ĸ����ֵ
		*/
		if(sum_output > (int32_t)remain_energy * (int32_t)remain_energy * (int32_t)RemainParam)
			P = (float)sum_output/(float)((int32_t)remain_energy * (int32_t)remain_energy * (int32_t)RemainParam);
		for(int i=0;i<4;i++)
			Chassis.M3508[i].Output = (float)Chassis.M3508[i].Output / P;
	}
	
}	
void Chassis_Process(RemoteData_t RDMsg,HolderData_t HDMsg)
{
    Chassis_GetMoveData(RDMsg,HDMsg);
    Chassis_LPfIn();
	  if(Observer.Tx.DR16_Rate>15)
		{
      Chassis_PidRun();
		}
		Chassis_PowerControl();
    Chassis_LPfOut();
    Chassis_CanTransmit();
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
