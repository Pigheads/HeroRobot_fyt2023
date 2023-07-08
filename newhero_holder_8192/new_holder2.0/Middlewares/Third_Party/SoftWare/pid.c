#include "pid.h"

/**
  * @brief  增量式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅，增量限幅.
  * @note   .
  * @retval none.
  */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim,float OutLim) //PID初始化系数
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
	  PID->OutLim = OutLim;
}

/**
  * @brief  普通的增量式PID（可以进行增量限幅）
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //误差量：目标-目前（相邻两次测量值取差分）

    dErrP = PID->errNow - PID->errOld1;					   //计算比例分量----微分：现误差-上一个误差
    dErrI = PID->errNow;								   //计算积分分量——————现误差
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //计算微分分量——————现误差-2*一阶误差微分+二阶误差微分

//		if (PID->errNow < 10 && PID->errNow > -10)
//			dErrI = 0;
		
    /*增量式PID计算*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID合成输出量

    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分


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
  * @brief  变PID参数
  * @param  kp最大，最小值 ki最大，最小值 PID结构体地址
  * @note   .
  * @retval none.
  */
void Holder_Pi_Tunning(float p_min,float p_max,float i_min, float i_max, PID_AntiIntegralType * PID)
{
	  float err = PID->errNow;
	  err = err<=0? -err : err;    /*< 取绝对值*/
	  PID->kp = p_min + p_max*(1 - 1/(1+0.01f*(err+err*err*err)));
    PID->ki = i_min + i_max*(1/(1+0.5f*err));   /*< 误差小，I大， 稳定性强 */
}

/**
  * @brief  位置式积分限幅+积分分离PID
  * @param  PID结构体地址，P,I,D,输出限幅，积分限幅，积分分离
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
  * @brief  抗积分饱和PID（I限幅）
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID)
{
	  PID->errNow = Target - Current;
	  PID->errNow_fabs=(PID->errNow>0)?(PID->errNow):(-PID->errNow);

    if(PID->I_Band!=0)
		{
	   if(PID->errNow_fabs<PID->I_Band)
      PID->errI = PID->errI + PID->ki* PID->errNow;	//积分计算，ki控制
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

		PID->errP = PID->kp *  PID->errNow;//比例计算，kp控制 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //微分计算，kd控制
	  
    PID->ctrOut=PID->errP+PID->errI+PID->errD;
    
    if (PID->ctrOut > PID->Out_Limit)
        PID->ctrOut = PID->Out_Limit;
    if (PID->ctrOut < -PID->Out_Limit)
        PID->ctrOut = -PID->Out_Limit; 

    PID->errOld = PID->errNow; //保存现在的误差
    return PID->ctrOut;
}

/**
  * @brief  三段绝对式PID初始化
  * @param  kp及差值err分段，ki，kd，限幅
  * @note   分段赋值
  * @retval 无
*/
void pid_init_absolute_threesection(PID_AbsoluteType_ThreeSection *PID,float *kp,float *err,float ki,float kd,float I_Limit,float Out_Limit)
{
	PID->tempkp[0]		= kp[0];
	PID->tempkp[1]		= kp[1];
	PID->tempkp[2]		= kp[2];
	PID->tempkp[3]		= kp[3];
	PID->err[0]   =  err[0];
	PID->err[1]   =  err[1];		
	PID->err[2]   =  err[2];	
	PID->err[3]   =  err[3];
	PID->ki		 = ki;
	PID->kd		 = kd;
	PID->I_Lim = I_Limit;
	PID->OUTMAX = Out_Limit;
	
}

/**
  * @brief 绝对式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅.
  * @note   .
  * @retval none.
  */
void pid_init_absolute(PID_AbsoluteType* PID,float kp, float ki, float kd, float errlimit)
{
	PID->kp		= kp;
	PID->ki		= ki;
	PID->kd		= kd;
	PID->errLim 	= errlimit;
	PID->errNow= 0;
	PID->errP= 0;
	PID->errI= 0;
	PID->errD= 0;
	PID->errOld= 0;
	PID->ctrOut= 0;
}

/**
  * @brief  普通绝对式PID.
  * @param  目标值，实际值，PID结构体地址.
  * @note   .
  * @retval 需要输出的值.
  */
float pid_absolute(float Target,float Current,PID_AbsoluteType* PID)
{
	PID->errNow = Target - Current;

	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow*0.3f;//0.4/误差积分，用于ki控制

	if(PID->errLim != 0)	   
	{
		if(PID->errI >  PID->errLim)  
			PID->errI =  PID->errLim;
		else if(PID->errI <  -PID->errLim)  
			PID->errI = -PID->errLim;
	}	
 
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

	PID->errOld = PID->errNow;	//保存现在的误差
 
	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出
	
	if(PID->ctrOut > 32760)
		PID->ctrOut = 32760;
	else if(PID->ctrOut < -32760)
		PID->ctrOut = -32760;
	return PID->ctrOut;
}

/**
  * @brief  三段绝对式PID
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
*/
float pid_absolute_threesection_update(float Target, float Current,PID_AbsoluteType_ThreeSection *PID)
{
	PID->errNow = Target - Current;
	
	if(PID->errNow > -PID->err[0] && PID->errNow < PID->err[0])
		PID->kp = PID ->errI = 0;
	else if(PID->errNow > -PID->err[1] && PID->errNow < PID->err[1])
		PID->kp = PID ->tempkp[0];
	else if(PID->errNow > -PID->err[2] && PID->errNow < PID->err[2])
		PID->kp = PID->tempkp[1];
	else if(PID->errNow > -PID->err[3] && PID->errNow < PID->err[3])
		PID->kp = PID->tempkp[2];
	else 
		PID->kp = PID->tempkp[3];
  
	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow; //误差积分，用于ki控制
	
	if(PID->I_Lim!=0)	   //积分上限和下限
	{
		if(PID->errI>PID->I_Lim)  
			PID->errI=PID->I_Lim;
		else if(PID->errI<-PID->I_Lim)  
			PID->errI=-PID->I_Lim;
	}
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制
	PID->errOld = PID->errNow;	//保存现在的误差

	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

	if (PID->ctrOut >  PID->OUTMAX)
      PID->ctrOut =  PID->OUTMAX;
  if (PID->ctrOut < -PID->OUTMAX)
      PID->ctrOut = -PID->OUTMAX; 

	return PID->ctrOut;
}


float pid_absolute_threesection_update_changed(float Target, float Current,PID_AbsoluteType_ThreeSection *PID)
{
	PID->errNow = Target - Current;
	
	if(PID->errNow > -PID->err[0] && PID->errNow < PID->err[0])
		PID->kp = PID ->tempkp[0];
	else if(PID->errNow > -PID->err[1] && PID->errNow < PID->err[1])
		PID->kp = PID->tempkp[1];
	else
		PID->kp = PID->tempkp[2];
  
	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow; //误差积分，用于ki控制
	
	if(PID->I_Lim!=0)	   //积分上限和下限
	{
		if(PID->errI>PID->I_Lim)  
			PID->errI=PID->I_Lim;
		else if(PID->errI<-PID->I_Lim)  
			PID->errI=-PID->I_Lim;
	}
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制
	PID->errOld = PID->errNow;	//保存现在的误差

	PID->ctrOut = 1.1f*(PID->kp) * PID->errP + 1.01f*(PID->ki) * PID->errI + 0.9f*(PID->kd) * PID->errD;//计算绝对式PID输出

	if (PID->ctrOut >  PID->OUTMAX)
      PID->ctrOut =  PID->OUTMAX;
  if (PID->ctrOut < -PID->OUTMAX)
      PID->ctrOut = -PID->OUTMAX; 

	return PID->ctrOut;
}

float pid_increment_update_changed(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //误差量：目标-目前（相邻两次测量值取差分）

    dErrP = PID->errNow - PID->errOld1;					   //计算比例分量----微分：现误差-上一个误差
    dErrI = PID->errNow;								   //计算积分分量——————现误差
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //计算微分分量——————现误差-2*一阶误差微分+二阶误差微分

    /*增量式PID计算*/
    PID->dCtrOut = 1.1f*(PID->kp) * dErrP + 1.01f*(PID->ki) * dErrI + 0.9f*(PID->kd) * dErrD; //PID合成输出量

    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分

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