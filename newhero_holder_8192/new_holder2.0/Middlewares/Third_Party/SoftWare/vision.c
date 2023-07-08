/**
  ******************************************************************************
  * @file    
  * @author  sy
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
#include "vision.h"
#include "string.h"
#include "judge.h"
#include "holder.h"
#include "message.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
vData_t vData = {0};	//�Ӿ�λ������

float auto_yaw_speed_kp = -0, auto_yaw_accel_kp = 0;	//�ٶ�����ٶȵ�Ԥ�����
float feed_pre_angle_yaw, predict_angle_yaw;			//Ԥ��ƫ����
float last_angle;
float auto_yaw_speed;

//�Ӿ��������ṹ��
kalman1_state vision_dis_pitch_offset;
kalman1_state vision_absolute_Yaw_Kal, vision_absolute_Pitch_Kal;
kalman1_state vision_angleY_KF, vision_angleP_KF;
kalman1_state vision_speedY_KF, vision_speedP_KF;
kalman1_state vision_accelY_KF, vision_accelP_KF;

kalman1_state vision_distance_KF, vision_speedD_KF;
moving_Average_Filter vision_angleY_MF, vision_angleP_MF, vision_RawangleY_MF,vision_absolute_Yaw_MF;
moving_Average_Filter vision_speedY_MF, vision_speedP_MF;
moving_Average_Filter zhen_time_ms;
/* function ------------------------------------------------------------------*/
void Vision_RecvData(uint8_t byte)
{
    vData.buf[vData.index] = byte;
    if (vData.buf[vData.index] == 0x0D && vData.buf[vData.index - 1] == 0x00)
    {
        if (vData.buf[vData.index - 15] == 0XFF && vData.buf[vData.index - 14] == 0X00)
        {
            memcpy(&vData.Pos, &vData.buf[vData.index - 13], 12);
				/**	x pitch 
					* y yaw
					* z ����
					*/
            vData.index = 0;
        }
    }
    vData.index++;
    if (vData.index == 200)
        vData.index = 0;
}

uint8_t id;
uint8_t shootSpeed;
uint8_t tmp_data[16] = {0}; 
float deg = 0;
void Vision_SendData(void)
{
    
		static uint16_t tx_tick_up = 0;
    static uint8_t color_state  = 0; 
    static uint8_t detect_state = 0;
    id = (uint8_t)JUDGE_u8GetRobotId();
		shootSpeed = (uint8_t)JUDGE_u16GetSpeedHeat42Limit();
    if (id <= 7 && id >0)
    {
        color_state = 1;
    }
    else if (id > 7)
    {
        color_state = 0;
    }
    
    switch(color_state)
    {
        case 0:
            tmp_data[1] = 0x00; /*< ��ɫ*/
        break;

        case 1:
            tmp_data[1] = 0x01; /*< ��ɫ*/
        break;
        
        default:
            tmp_data[1] = 0x00; /*< ��ɫ*/
        break;
    }

    /* ֡ͷ֡β */
    tmp_data[0] = 0xFF;
//		tmp_data[1] = 0x01;
		/* �ӵ����� */
		float shoot_speed = 16.0;
		memcpy(&tmp_data[2], &shoot_speed, sizeof(shoot_speed));
//		float pitch = Holder.Pitch._0x20A.Rx.Angle;
//		pitch = (pitch - 5900)/3600 * 180 ;
		
		float pitch = mc_imu_pitch.angle-180;
		if(pitch>0)
			pitch-=180;
		else if(pitch<0)
			pitch+=180;
//		pitch -= 20;
//		deg = pitch;
		
		memcpy(&tmp_data[6], &pitch, sizeof(pitch));
//		float yaw = Holder.Yaw._0x209.Rx.Angle;
//		if(yaw>180)yaw = yaw-360;
//		yaw = -yaw;
		float yaw = -mc_imu_yaw.angle;
		memcpy(&tmp_data[10], &yaw, sizeof(yaw));
		
    tmp_data[14] = 0x00;
    tmp_data[15] = 0x0D;
		
	  HAL_UART_Transmit_IT(&huart6, tmp_data, 16);
}
// kalman�˲�
/**
  * @brief    average_init
  * @note    �����˲�����ʼ�������ó���
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_init(moving_Average_Filter *Aver, uint8_t lenth)
{
	uint16_t i;
	
	for(i = 0; i<MAF_MaxSize; i++)
		Aver->num[i] = 0;
	
	if(lenth >MAF_MaxSize)
	{
		lenth = MAF_MaxSize;
	}
	
	Aver->lenth = lenth;
	Aver->pot = 0;
	Aver->aver_num = 0;
	Aver->total = 0;
}

/**
  * @brief    average_add
  * @note    ����ƽ���˲���������У��Ƚ��ȳ�
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{
	
	Aver->total -=  Aver->num[Aver->pot];
	Aver->total += add_data;
	
	Aver->num[Aver->pot] = add_data;
	
	Aver->aver_num = (Aver->total)/(Aver->lenth);
	Aver->pot++;
	
	if(Aver->pot == Aver->lenth)
	{
		Aver->pot = 0;
	}
}

/**
  * @brief    average_get
  * @note    ��ȡ��ǰpre�ε����ݣ�����������鳤����ȡ��¼�����������
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
float average_get(moving_Average_Filter *Aver, uint16_t pre)
{
	float member;
	uint8_t temp;
	
	if(Aver->pot != 0)
	{
		temp = Aver->pot-1;
	}
	else 
	{
		temp = Aver->lenth-1;
	}
	
	if(pre>Aver->lenth)
		pre = pre % Aver->lenth;
	
	if(pre>temp)
	{
		pre = Aver->lenth+temp-pre;
	}
	else
	{
		pre = temp-pre;
	}
	
	member = Aver->num[pre];
	
	return member;
}

void vision_init()
{
	kalman1_init(&vision_absolute_Yaw_Kal, 0.1, 10);//10);	//���ԽǶȵĿ������˲���
	kalman1_init(&vision_absolute_Pitch_Kal,1, 20);
	kalman1_init(&vision_dis_pitch_offset, 1, 30);
	
	kalman1_init(&vision_angleY_KF, 0.2, 50);
	kalman1_init(&vision_angleP_KF, 1, 0);
	kalman1_init(&vision_speedY_KF, 0.2, 20);//5);
	kalman1_init(&vision_speedP_KF, 1, 40);
	kalman1_init(&vision_accelY_KF, 1, 10);
	kalman1_init(&vision_accelP_KF, 1, 0);
	kalman1_init(&vision_distance_KF, 1, 30);
	kalman1_init(&vision_speedD_KF, 1, 100);
	
	average_init(&vision_angleY_MF, 50);
	average_init(&vision_speedY_MF, 100);
	average_init(&vision_RawangleY_MF, 50);
	average_init(&vision_absolute_Yaw_MF, 100);
//	average_init(&vision_angleY_MF, 50);
}

void Holder_CoordinateTransform(void)
{
	//�任��ֱ������ϵ
	float x,y,z;
	x = -vData.Pos.z * sinf(vData.Pos.y*PI/180.0f);
	y = vData.Pos.z * cosf(vData.Pos.y*PI/180.0f) * sin(vData.Pos.x*PI/180.0f);
	z = vData.Pos.z * cosf(vData.Pos.y*PI/180.0f) * cosf(vData.Pos.x*PI/180.0f);
	
	//�������
	x-=CAMERA_X_DEVIATION;
	y-=CAMERA_Y_DEVIATION;
	z-=CAMERA_Z_DEVIATION;
	
	vData.Pos.z = quakeSqrt(x*x+y*y+z*z);
	vData.Pos.y = atan(x/z)*180.0f/PI;
	vData.Pos.x = asin(y/vData.Pos.z)*180.0f/PI;
}

float yaw_angle_raw;
float auto_yaw_angle =0,yaw_speed_raw=0;
float predict_angle_yaw, predict_angle_pitch, predict_angle_distance;	//Ԥ��
void vision_predict(void)
{
	yaw_angle_raw = vData.Pos.x*8192.0f/360.0f + vData.cur_gyro_yaw;

	//����ʱ��Ҫ�����ԽǶ��Ƿ��б仯����������£�Ŀ�겻����������Ľ��Ӧ���ǲ����
	//���ԽǶ��˲�

	average_add(&vision_angleY_MF, yaw_angle_raw);
	
	auto_yaw_angle = kalman1_filter(&vision_angleY_KF, vision_angleY_MF.aver_num);
	
	average_add(&vision_RawangleY_MF, auto_yaw_angle);
	
	float yaw_speed_raw =  (auto_yaw_angle - average_get(&vision_RawangleY_MF, 5))*10.0f;//���˲������ٶ�
	
	last_angle = auto_yaw_angle;
	
	if(vData.Pos.z == -1)//��ʧ����װ��0�������ٶ�ͻ��
	{
		for(int i = 0; i<vision_speedY_MF.lenth; i++)
		{
			average_add(&vision_speedY_MF, 0);
		}
	}
	
//	average_add(&vision_speedY_MF, yaw_speed_raw);
	
	auto_yaw_speed = kalman1_filter(&vision_speedY_KF, yaw_speed_raw);
	
	auto_yaw_speed = LIMIT(auto_yaw_speed,-500,500);//Ԥ�����޷�	
	
	feed_pre_angle_yaw = auto_yaw_speed + auto_yaw_angle;
	
	

	//��ͨԤ��
//	predict_angle_yaw = predict_angle_yaw+feed_pre_angle_yaw;
	predict_angle_yaw =  kalman1_filter(&vision_absolute_Yaw_Kal, feed_pre_angle_yaw);	

	average_add(&vision_absolute_Yaw_MF, predict_angle_yaw);
	
	predict_angle_yaw = vision_absolute_Yaw_MF.aver_num;

	UART_SendWave(2, 4, &yaw_angle_raw, &auto_yaw_speed);
//	UART_SendWave(3, 4, &yaw_angle_raw, &auto_yaw_speed, &vision_absolute_Yaw_MF.aver_num);
	
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
