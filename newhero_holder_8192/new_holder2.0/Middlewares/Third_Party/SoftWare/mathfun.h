#ifndef _MATHFUN_FUNC
#define _MATHFUN_FUNC
#include <stdint.h>
#define LIMIT(IN,MIN,MAX)	(IN <= MIN ? MIN : (IN >= MAX ? MAX : IN))
#define ABS(IN) (IN>=0 ? IN : -IN)
float cos_x(int16_t angle);
float sin_x(int16_t angle);

int32_t diff(int32_t Rx);
int16_t Receive_filter_4(int16_t receive_data,int16_t *Filter_Buf); //ËÄµãÂË²¨
int32_t Receive_filter_4_32(int32_t receive_data,int32_t *Filter_Buf);
int16_t Pitch_Rx_Lpf(int16_t RxAngle);
int16_t Yaw_Rx_Lpf(int16_t RxAngle);
float quakeSqrt(float f_num);
#endif

