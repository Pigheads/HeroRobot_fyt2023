//#include "adc.h"
//#include "ccd.h"
//#include "usart.h"
//int TIME_us=10;
//uint8_t ccd_adc[128]={0};
//uint8_t SciBuf[200]={0};

//void RD_TSL(void) 
// {
//		u8 i=0,tslp=0;
//		TSL_CLK=1;           
//		TSL_SI=0; 
//		HAL_Delay(TIME_us);
//				
//		TSL_SI=1; 
//		TSL_CLK=0;
//		HAL_Delay(TIME_us);
//				
//		TSL_CLK=1;
//		TSL_SI=0;
//		HAL_Delay(TIME_us); 
//		for(i=0;i<128;i++)
//		{ 
//			TSL_CLK=0; 
//			HAL_Delay(TIME_us);  
//			ccd_adc[tslp]=(u8)((float)Get_Adc(ADC_Channel_1)/4096*255); 
//			++tslp;
//			TSL_CLK=1;
//			HAL_Delay(TIME_us);
//		}  
// }

//void slove_data(void)
// {
//		int i;
////	RD_TSL();
//    SciBuf[0] = 0; 
//	  SciBuf[1] = 132;
//    SciBuf[2] = 0; 
//    SciBuf[3] = 0;
//	  SciBuf[4] = 0;
//    SciBuf[5] = 0; 
//		for(i=0;i<128;i++)
//			SciBuf[6+i] = ccd_adc[i];
// }
//	
//void SendToPc(void)
// { 
//	  uint8_t *data;
//	  data=SciBuf;
//	  uint8_t lrc=0;
//	  PutChar('*');
//    int len=(int)(data[0]<<8) | (int)(data[1]);
//    data+=2;
//    PutChar('L');
//	  PutChar('D');
//	  while(len--)
//		{
//			SendHex(*data);
//			lrc+=*data++;
//		}
//		lrc=0-lrc;
//		SendHex(lrc);
//		PutChar('#');
// }	
// 
// void SendHex(uint8_t data)
// {
//	  uint8_t  temp; 
//    temp = data >> 4; 
//    if (temp >= 10) 
//    { 
//      PutChar(temp - 10 + 'A'); 
//    } 
//    else 
//    { 
//      PutChar(temp + '0'); 
//    } 
//    temp = data & 0x0F; 
//    if (temp >= 10) 
//    { 
//      PutChar(temp - 10 + 'A'); 
//    } 
//    else 
//    { 
//			PutChar(temp + '0'); 
//    } 
// }
// 
// void PutChar(uint8_t data)
// {
//	 uint8_t *p;
//	 *p=data;
//	 HAL_UART_Transmit(&huart2,p,1,20);
// }
// 
// 