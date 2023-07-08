#include "pid.h"

/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷��������޷�.
  * @note   .
  * @retval none.
  */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim,float OutLim) //PID��ʼ��ϵ��
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
	  PID->OutLim = OutLim;
}

/**
  * @brief  ��ͨ������ʽPID�����Խ��������޷���
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //�������Ŀ��-Ŀǰ���������β���ֵȡ��֣�

    dErrP = PID->errNow - PID->errOld1;					   //�����������----΢�֣������-��һ�����
    dErrI = PID->errNow;								   //������ַ��������������������
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //����΢�ַ��������������������-2*һ�����΢��+�������΢��

    /*����ʽPID����*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID�ϳ������

    PID->errOld2 = PID->errOld1; //�������΢��
    PID->errOld1 = PID->errNow;  //һ�����΢��

    if (PID->dCtrOut < -PID->IncLim)
        PID->dCtrOut = -PID->IncLim;
    else if (PID->dCtrOut > PID->IncLim)
        PID->dCtrOut = PID->IncLim;

    PID->ctrOut += PID->dCtrOut;
		if(PID->ctrOut>PID->OutLim)
			PID->ctrOut=PID->OutLim;
		if(PID->ctrOut<-PID->OutLim)
			PID->ctrOut=-PID->OutLim;
    return PID->ctrOut;
}

/**
  * @brief  ��PID����
  * @param  kp�����Сֵ ki�����Сֵ PID�ṹ���ַ
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning(float p_min,float p_max,float i_min, float i_max, PID_AntiIntegralType * PID)
{
	  float err = PID->errNow;
	  err = err<=0? -err : err;    /*< ȡ����ֵ*/
	  PID->kp = p_min + p_max*(1 - 1/(1+0.01f*(err+err*err*err)));
    PID->ki = i_min + i_max*(1/(1+0.5f*err));   /*< ���С��I�� �ȶ���ǿ */
}

/**
  * @brief  λ��ʽ�����޷�+���ַ���PID
  * @param  PID�ṹ���ַ��P,I,D,����޷��������޷������ַ���
  * @note   .
  * @retval none.
  */
void pid_init_antiintegral(PID_AntiIntegralType *PID, float kp, float ki, float kd, float Out_Limit,float I_Limit,float I_Band)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->Out_Limit = Out_Limit;
    PID->I_Limit = I_Limit;
	  PID->I_Band = I_Band;
}

/**
  * @brief  �����ֱ���PID��I�޷���
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID)
{
	  PID->errNow = Target - Current;
	  PID->errNow_fabs=(PID->errNow>0)?(PID->errNow):(-PID->errNow);

    if(PID->I_Band!=0)
		{
	   if(PID->errNow_fabs<PID->I_Band)
      PID->errI = PID->errI + PID->ki* PID->errNow;	//���ּ��㣬ki����
		 else
			PID->errI = 0;
	  }
    else
		{
			PID->errI = PID->errI + PID->ki* PID->errNow;
		}
		
	  if(PID->I_Limit!=0)
		{
			if(PID->errI > PID->I_Limit)	
         PID->errI = PID->I_Limit;
      if(PID->errI < -PID->I_Limit)
         PID->errI = -PID->I_Limit;
    }

		PID->errP = PID->kp *  PID->errNow;//�������㣬kp���� 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //΢�ּ��㣬kd����
	  
    PID->ctrOut=PID->errP+PID->errI+PID->errD;
    
    if (PID->ctrOut > PID->Out_Limit)
        PID->ctrOut = PID->Out_Limit;
    if (PID->ctrOut < -PID->Out_Limit)
        PID->ctrOut = -PID->Out_Limit; 

    PID->errOld = PID->errNow; //�������ڵ����
    return PID->ctrOut;
}

/**
  * @brief  ���ξ���ʽPID��ʼ��
  * @param  kp����ֵerr�ֶΣ�ki��kd���޷�
  * @note   �ֶθ�ֵ
  * @retval ��
*/
void pid_init_absolute_threesection(PID_AbsoluteType_ThreeSection *PID,float *kp,float *err,float ki,float kd,float I_Limit,float Out_Limit)
{
	PID->tempkp[0]		= kp[0];
	PID->tempkp[1]		= kp[1];
	PID->tempkp[2]		= kp[2];
	PID->err[0]   =  err[0];
	PID->err[1]   =  err[1];		
	PID->err[2]   =  err[2];	
	PID->ki		 = ki;
	PID->kd		 = kd;
	PID->I_Lim = I_Limit;
	PID->OUTMAX = Out_Limit;
}

/**
  * @brief  ���ξ���ʽPID
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
*/
float pid_absolute_threesection_update(float Target, float Current,PID_AbsoluteType_ThreeSection *PID)
{
	PID->errNow = Target - Current;
	if(PID->errNow > -0 && PID->errNow < 0)
		PID->kp = PID ->errI = 0;
	else if(PID->errNow > -PID->err[0] && PID->errNow < PID->err[0])
		PID->kp = PID ->tempkp[0];
	else if(PID->errNow > -PID->err[1] && PID->errNow < PID->err[1])
		PID->kp = PID->tempkp[1];
	else if(PID->errNow > -PID->err[2] && PID->errNow < PID->err[2])
		PID->kp = PID->tempkp[2];
	else PID->kp = 1.1;
  
	PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����
	
	PID->errI += PID->errNow; //�����֣�����ki����
	
	if(PID->I_Lim!=0)	   //�������޺�����
	{
		if(PID->errI>PID->I_Lim)  
			PID->errI=PID->I_Lim;
		else if(PID->errI<-PID->I_Lim)  
			PID->errI=-PID->I_Lim;
	}
	PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����
	PID->errOld = PID->errNow;	//�������ڵ����

	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

	if (PID->ctrOut >  PID->OUTMAX)
      PID->ctrOut =  PID->OUTMAX;
  if (PID->ctrOut < -PID->OUTMAX)
      PID->ctrOut = -PID->OUTMAX; 

	return PID->ctrOut;
}
