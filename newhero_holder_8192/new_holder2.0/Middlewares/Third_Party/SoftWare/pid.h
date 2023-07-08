#ifndef __PID_H
#define __PID_H
#include "stdio.h"
typedef struct
{
    /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
    float kp; //����ϵ��
    float ki; //����ϵ��
    float kd; //΢��ϵ��

    float errNow;  //��ǰ�����
    float dCtrOut; //�����������
    float ctrOut;  //�������

    float IncLim; //�����޷�
	  float OutLim; 
    /*PID�㷨�ڲ���������ֵ�����޸�*/
    float errOld1;
    float errOld2;

} PID_IncrementType;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
    float ctrOut;
    float Out_Limit;
	  float I_Limit;
	  float I_Band;
	  float errNow_fabs;
} PID_AntiIntegralType; /* ���ַ��룬�����ֱ���pid */

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float errLim;//��������
	float errNow;
	float errOld;
	float errP;
	float errI;
	float errD;
	float ctrOut;
} PID_AbsoluteType;

typedef struct 
{
	float tempkp[4];
	float err[4];
	float kp;
	float ki;
	float kd;
	float I_Lim;
	float errNow;
	float errOld;
	float errP;
	float errI;
	float errD;
	float ctrOut;
	float OUTMAX;
} PID_AbsoluteType_ThreeSection;/*���ξ���ʽPID*/

void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim,float OutLim);
void pid_init_antiintegral(PID_AntiIntegralType *PID, float kp, float ki, float kd, float Out_Limit,float I_Limit,float I_Band);
void pid_init_absolute_threesection(PID_AbsoluteType_ThreeSection *PID,float *kp,float *err,float ki,float kd,float I_Limit,float Out_Limit);
void pid_init_absolute(PID_AbsoluteType* PID,float kp, float ki, float kd, float errlimit);

float pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID);
float pid_increment_update(float Target, float Current, PID_IncrementType *PID);
float pid_absolute_threesection_update(float Target, float Current,PID_AbsoluteType_ThreeSection *PID);
float pid_absolute(float Target,float Current,PID_AbsoluteType* PID);
void Holder_Pi_Tunning(float p_change,float p_min,float i_change, float i_min, PID_AntiIntegralType *PID);

float pid_absolute_threesection_update_changed(float Target, float Current,PID_AbsoluteType_ThreeSection *PID);
float pid_increment_update_changed(float Target, float Current, PID_IncrementType *PID);
#endif
