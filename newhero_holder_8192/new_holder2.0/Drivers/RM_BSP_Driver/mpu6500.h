/**
  ******************************************************************************
  * @file    
  * @author  v1.0wsh v2.0sy
  * @brief   
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MPU6500_H
#define _MPU6500_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "spi.h"
#include "tim.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _MPU_Value_t
{
    int16_t Accel[3];
    int16_t Gyro[3];
}MPU_Value_t;
/* define --------------------------------------------------------------------*/
#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_CONFIG_2  0x1D

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75

#define MPU6500_DISABLE     HAL_GPIO_WritePin(MPU_NSS_GPIO_Port,MPU_NSS_Pin,GPIO_PIN_SET)
#define MPU6500_ENABLE      HAL_GPIO_WritePin(MPU_NSS_GPIO_Port,MPU_NSS_Pin,GPIO_PIN_RESET)
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void MPU6500_GyroInit(void);
void MPU6500_GyroUpdate(void);

extern MPU_Value_t MPU_Value;

#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
