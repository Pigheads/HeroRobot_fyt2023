#include "mathfun.h"

float _fast_cos[91] = { 1,
0.999848,0.999391,0.99863,0.997564,0.996195,0.994522,0.992546,0.990268,0.987688,0.984808,
0.981627,0.978148,0.97437,0.970296,0.965926,0.961262,0.956305,0.951057,0.945519,0.939693,
0.93358,0.927184,0.920505,0.913545,0.906308,0.898794,0.891007,0.882948,0.87462,0.866025,
0.857167,0.848048,0.838671,0.829038,0.819152,0.809017,0.798635,0.788011,0.777146,0.766044,
0.75471,0.743145,0.731354,0.71934,0.707107,0.694658,0.681998,0.669131,0.656059,0.642788,
0.62932,0.615661,0.601815,0.587785,0.573576,0.559193,0.544639,0.529919,0.515038,0.5,
0.48481,0.469471,0.45399,0.438371,0.422618,0.406737,0.390731,0.374606,0.358368,0.34202,
0.325568,0.309017,0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,
0.156434,0.139173,0.121869,0.104528,0.0871556,0.0697563,0.0523358,0.0348993,0.0174522,-1.73205e-07};

float cos_x(int16_t angle)
{
	if (angle>=0 && angle <= 90)
	{
		return _fast_cos[angle];
	}
	else if (angle > 90 && angle <=180)
	{
		return -(_fast_cos[180-angle]);
	}
	else if (angle > 180 && angle <=360)
	{
		return cos_x(360-angle);
	}
	else if (angle > 360)
	{
		return cos_x(angle - 360);
	}
	else if (angle < 0)
	{
		return (cos_x(-angle));
	}
	return 0;
}

float sin_x(int16_t angle)
{
	return cos_x(angle - 90);
}

int32_t Rx_old;
int32_t diff(int32_t Rx)
{
	   int32_t diff_rx;
	   diff_rx=Rx-Rx_old;
	   Rx_old=Rx;
	   return diff_rx;
}
/**
  * @brief  Quake-III Arena
  * @param  x
  * @retval 根号x
  * @attention  大神写的快速平方根    
  */
float quakeSqrt(float f_num)
{
    if(f_num == 0) return 0; 
    float result = f_num; 
    float num_half = 0.5f*result; 
    int i = *(int*)&result; 

    i = 0x5f3759df - (i>>1); 
    result = *(float*)&i; 
    result = result*(1.5f-num_half*result*result);
    result = result*(1.5f-num_half*result*result); 
    return 1.0f/result; 
}

//简单滤波器

/*
  四段滤波
*/
int16_t Receive_filter_4(int16_t receive_data,int16_t *Filter_Buf)
{ 
	int16_t sum=0;
	for(uint8_t i=0;i<3;i++)
	  Filter_Buf[i]=Filter_Buf[i+1];
	  Filter_Buf[3]=receive_data;

	sum=Filter_Buf[0]*0.1+Filter_Buf[1]*0.2+Filter_Buf[2]*0.3+Filter_Buf[3]*0.4;
	return sum;
}

int32_t Receive_filter_4_32(int32_t receive_data,int32_t *Filter_Buf)
{ 
	int32_t sum=0;
	for(uint8_t i=0;i<3;i++)
	  Filter_Buf[i]=Filter_Buf[i+1];
	  Filter_Buf[3]=receive_data;

	sum=Filter_Buf[0]*0.1+Filter_Buf[1]*0.2+Filter_Buf[2]*0.3+Filter_Buf[3]*0.4;
	return sum;
}
/*
  俯仰电机角度滤波
*/
int16_t RxPitchAngle_old;
int16_t Pitch_Rx_Lpf(int16_t RxAngle)
{ 
	int16_t d;
	d=RxAngle-RxPitchAngle_old;
  if(d<-1||d>1)
		RxPitchAngle_old=RxAngle;
	return RxPitchAngle_old;
}
/*
  偏航电机角度滤波
*/
int16_t RxYawAngle_old;
int16_t Yaw_Rx_Lpf(int16_t RxAngle)
{ 
	int16_t d;
	d=RxAngle-RxYawAngle_old;
  if(d<-1||d>1)
		RxYawAngle_old=RxAngle;
	return RxYawAngle_old;
}