/**
  ******************************************************************************
  * @file    
  * @author  v1.0hjh v2.0wsh
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
#include "cmsis_os.h"
#include "math.h"
#include "adc.h"
#include "key.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define FIFOSIZE 20
/* variables -----------------------------------------------------------------*/
KeyMsg_t KeyMsg[FIFOSIZE] = {Max,0};
uint8_t key_msg_rear = 0;
uint8_t key_msg_front = 0;
uint8_t key_msg_flag = 0;
uint8_t KeyTime[Max] = {0};
/* function ------------------------------------------------------------------*/

uint16_t ADC_Get(void)   
{
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY) == HAL_OK)
    {
        return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}

GPIO_PinState ADC_ReadVol(float level)
{
	float KeyTmp = (float)(ADC_Get()*(3.3f/4096));
	osDelay(3);
    float KeyTmp2 = (float)(ADC_Get()*(3.3f/4096));
	if(fabs(KeyTmp-level)<0.1&&fabs(KeyTmp2-level)<0.1)
		return GPIO_PIN_RESET;
	else
		return GPIO_PIN_SET;  
}

GPIO_PinState Key_Get(Key_e key)
{
    GPIO_PinState key_state;
    switch(key)
    {
        case Up:
            key_state = KEY_UP;
            break;
        case Down:
            key_state = KEY_DOWN;
            break;
        case Back:
            key_state = KEY_BACK;
            break;
        case Enter:
            key_state = KEY_ENTER;
            break;
        case Max:
            break;
    }
    return key_state;    
}

void SendKeyMsgToFifo(KeyMsg_t key_msg)
{
    uint8_t temp = 0;
    if(key_msg_flag == 1)
    {
        return ;
    }
    KeyMsg[key_msg_rear].Key = key_msg.Key;
    KeyMsg[key_msg_rear].Status = key_msg.Status;
    key_msg_rear++;
    if(key_msg_rear >= FIFOSIZE)
    {
        key_msg_rear = 0;
    }
    temp = key_msg_rear;
    if(temp == key_msg_front)
    {
        key_msg_flag = 1;
    }
    else
    {
        key_msg_flag = 2;
    }    
}

uint8_t GetKeyMsgFromFifo(KeyMsg_t *key_msg)
{
    uint8_t temp;

    if(key_msg_flag == 0)
    {
        return 0;
    }
    key_msg->Key = KeyMsg[key_msg_front].Key;
    key_msg->Status = KeyMsg[key_msg_front].Status;
    key_msg_front++;
    if(key_msg_front >= FIFOSIZE)
    {
        key_msg_front = 0;
    }
    temp = key_msg_rear;
    if(key_msg_front == temp)
    {
        key_msg_flag = 0;
    }
    else
    {
        key_msg_flag = 2;
    }
    return 1;
}

void Key_Process(void)
{
    Key_e key;
    GPIO_PinState key_status = GPIO_PIN_SET;
    KeyMsg_t key_msg;

    for(key = Up; key < Max; key++)
    {
        key_status = Key_Get(key);
        if(key_status == GPIO_PIN_RESET)
        {
            KeyTime[key]++;
            if(KeyTime[key] < 2)
            {
                continue;
            }
            else if(KeyTime[key] == 2)
            {
                key_msg.Key = key;
                key_msg.Status = 0;
                SendKeyMsgToFifo(key_msg);
            }
            else if(KeyTime[key] < 40)
            {
                continue;
            }
            else if(KeyTime[key] >= 40)
            {
                key_msg.Key = key;
                key_msg.Status = 1;
                SendKeyMsgToFifo(key_msg);
                KeyTime[key] = 0;
            }
        }
        else
        {
            if(KeyTime[key] >= 2)
            {
                key_msg.Key = key;
                key_msg.Status = 2;
                SendKeyMsgToFifo(key_msg);
                KeyTime[key] = 0;
            }
        }
    }
}


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
