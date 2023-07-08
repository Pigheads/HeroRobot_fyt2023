/**
  ******************************************************************************
  * @file    
  * @author  v1.0hjh v2.0zjy v3.0wsh v4.0sy
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
#include "ui.h"
#include "tim.h"
/* typedef -------------------------------------------------------------------*/
Page_t PageMain;
Page_t PageCan;
Page_t PageSensor;
Page_t PageChange;
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/

void CanData(void);
void CanData_CallBack(void);
void SensorData(void);
void SensorData_CallBack(void);
void ChangeParam(void);
uint16_t pluse = 200;
void UI_Init(void)
{
    uiPageMainDef(PageMain,Function,3);  
    uiPageFuncDef(PageMain,0,"CanTRx_Data",CanData);
    uiPageFuncDef(PageMain,1,"Sensor_Data",SensorData);
    uiPageFuncDef(PageMain,2,"Change_Param",ChangeParam);
    
    uiPageDef(PageCan,ShowStr,0);
    uiPageShowDef(PageCan,CanData_CallBack);
    
    uiPageDef(PageSensor,ShowStr,0);
    uiPageShowDef(PageSensor,SensorData_CallBack);
    
    uiPageDef(PageChange,ChangePara,1);
    uiPageParamDef(PageChange,0,"Snail_Pluse",pluse,UINT16);
    
}

void UI_Process(void)
{
    UI_Show(&PageMain);
}

void CanData(void)
{
    UI_Show(&PageCan);
}

void CanData_CallBack(void)
{
    OLED_printf(0,0,"CAN1Tx:%d",Observer.Tx.CAN1_Tx_Rate);
    OLED_ShowString(0,1,(uint8_t*)"CAN1Rx");
    OLED_printf(0,2,"0x206:%d",Observer.Tx.CAN_0x206_Rate);
    OLED_printf(0,3,"0x209:%d",Observer.Tx.CAN_0x209_Rate);
    
    OLED_printf(64,0,"CAN2Tx:%d",Observer.Tx.CAN2_Tx_Rate);
    OLED_ShowString(64,1,(uint8_t*)"CAN2Rx");
    OLED_printf(64,2,"0x201:%d",Observer.Tx.CAN_0x201_Rate);
    OLED_printf(64,3,"0x202:%d",Observer.Tx.CAN_0x202_Rate);
    OLED_printf(64,4,"0x203:%d",Observer.Tx.CAN_0x203_Rate);
    OLED_printf(64,5,"0x204:%d",Observer.Tx.CAN_0x204_Rate); 
    OLED_printf(64,6,"0x20A:%d",Observer.Tx.CAN_0x20A_Rate);     
}

void SensorData(void)
{
    UI_Show(&PageSensor);
}

void SensorData_CallBack(void)
{
    OLED_printf(0,0,"HI216_Eular:%0.2f",Observer.Tx.HI216_Value);
    OLED_printf(0,1,"MPU6500_Gyro:%d",Observer.Tx.MPU6500_Value);
    OLED_printf(0,2,"DR16_Rate:%d",Observer.Tx.DR16_Rate);
    OLED_printf(0,3,"CAMERA_Fps:%d",Observer.Tx.CAMERA_Rate);
    OLED_printf(0,4,"Chassis_Msg:%d",Observer.Tx.ChassisMsg_Rate);
    OLED_printf(0,5,"Holder_Msg:%d",Observer.Tx.HolderMsg_Rate);
}

void ChangeParam(void)
{
    UI_Show(&PageChange);
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
