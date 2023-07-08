#ifndef _MATHFUN_FUNC
#define _MATHFUN_FUNC

#include <stdint.h>

float cos_x(int16_t angle);
float sin_x(int16_t angle);

int32_t diff(int32_t Rx);
int16_t Receive_filter_4(int16_t receive_data,int16_t *Filter_Buf); //ËÄµãÂË²¨
int32_t Receive_filter_4_32(int32_t receive_data,int32_t *Filter_Buf);
int16_t Pitch_Rx_Lpf(int16_t RxAngle);
int16_t Ramp_function(int16_t delta_Min, int16_t target, int16_t now, int16_t ramp_Coeff);
#endif

