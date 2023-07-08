#ifndef __PID_H
#define __PID_H
#include "stdio.h"
typedef struct
{
    /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
    float kp; //比例系数
    float ki; //积分系数
    float kd; //微分系数

    float errNow;  //当前的误差
    float dCtrOut; //控制增量输出
    float ctrOut;  //控制输出

    float IncLim; //增量限幅
	  float OutLim; 
    /*PID算法内部变量，其值不能修改*/
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
} PID_AntiIntegralType; /* 积分分离，抗积分饱和pid */

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float errLim;//积分上限
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
} PID_AbsoluteType_ThreeSection;/*三段绝对式PID*/

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
