/**
  ******************************************************************************
  * @file    
  * @author  v1.0wsh v2.0sy
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
#include "tim.h"
#include "mpu6500.h"
#include "cmsis_os.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
uint8_t MPU6500_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    MPU6500_ENABLE;  
    status = SPI1_ReadWriteByte(reg); 
    SPI1_ReadWriteByte(value);
    MPU6500_DISABLE;
    TIM7_Delayus(1);
    return status;
}

uint8_t MPU6500_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    MPU6500_ENABLE;
    SPI1_ReadWriteByte(reg | 0x80);
    reg_val = SPI1_ReadWriteByte(0xff); 
    MPU6500_DISABLE;
    TIM7_Delayus(1);
    return reg_val;
}

void MPU6500_Init(void)
{
    MPU6500_Write_Reg(PWR_MGMT_1, 0x00);
    MPU6500_Write_Reg(CONFIG, 0x07); 
    MPU6500_Write_Reg(SMPLRT_DIV, 0x07);  
    MPU6500_Write_Reg(GYRO_CONFIG, 0x13); 
    MPU6500_Write_Reg(ACCEL_CONFIG, 0x18);
    MPU6500_Write_Reg(ACCEL_CONFIG_2, 0x08);
    HAL_Delay(50);
}

static uint8_t buf[6];
MPU_Value_t MPU_Value = {0};
void MPU6500_ReadAccel(void)
{
    buf[0] = MPU6500_ReadReg(ACCEL_XOUT_L);
    buf[1] = MPU6500_ReadReg(ACCEL_XOUT_H);
    MPU_Value.Accel[0] = buf[1]<<8|buf[0];
    MPU_Value.Accel[0] /= 164;
    
    buf[2] = MPU6500_ReadReg(ACCEL_YOUT_L);
    buf[3] = MPU6500_ReadReg(ACCEL_YOUT_H);
    MPU_Value.Accel[1] = buf[3]<<8|buf[2];
    MPU_Value.Accel[1] /= 164;
    
    buf[4] = MPU6500_ReadReg(ACCEL_ZOUT_L);
    buf[5] = MPU6500_ReadReg(ACCEL_ZOUT_H);
    MPU_Value.Accel[2] = buf[5]<<8|buf[4];
    MPU_Value.Accel[2] /= 164;
}

void MPU6500_ReadGyro(void)
{
    buf[0] = MPU6500_ReadReg(GYRO_XOUT_L);
    buf[1] = MPU6500_ReadReg(GYRO_XOUT_H);
    MPU_Value.Gyro[0] =	(buf[1] << 8) | buf[0];
				
    buf[2] = MPU6500_ReadReg(GYRO_YOUT_L);
    buf[3] = MPU6500_ReadReg(GYRO_YOUT_H);
    MPU_Value.Gyro[1] =	(buf[3] << 8) | buf[2];

    buf[4] = MPU6500_ReadReg(GYRO_ZOUT_L);
    buf[5] = MPU6500_ReadReg(GYRO_ZOUT_H);
    MPU_Value.Gyro[2] =	(buf[5] << 8) | buf[4];
}
static int32_t Gyro_Offset = 0;
void MPU6500_ReadOffset(void)
{
	uint16_t i;
	for (i = 0; i < 300; i++)
	{
		MPU6500_ReadGyro();
		Gyro_Offset += MPU_Value.Gyro[2];
	}
	Gyro_Offset /= 300;    
}

void MPU6500_GyroInit(void)
{
    MPU6500_Init();
    MPU6500_ReadOffset();
}

void MPU6500_GyroUpdate(void)
{
    MPU6500_ReadGyro();
    MPU_Value.Gyro[2] -= Gyro_Offset;
    Observer.Rx.MPU6500_Value = MPU_Value.Gyro[2];
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
