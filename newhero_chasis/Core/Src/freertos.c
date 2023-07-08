/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "message.h"
#include "holder.h"
#include "chassis.h"
#include "ui.h"
#include "keyboard.h"
#include "pid.h"
#include "can.h"
#include "judge.h"
#include "ammunition.h"
#include "frictiongear.h"
#include "vision.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osEvent evt;
osEvent evt1;
/* USER CODE END Variables */
osThreadId LEDHandle;
osThreadId MSGHandle;
osThreadId UIHandle;
osThreadId KEYHandle;
osThreadId ChassisHandle;
osThreadId HolderHandle;
osThreadId JudgeHandle;
osThreadId ShootHandle;
osMessageQId RDtHMsgHandle;
osMessageQId RDtCMsgHandle;
osMessageQId RDtSMsgHandle;
osMessageQId HDtHMsgHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskLED(void const * argument);
void taskMsg(void const * argument);
void taskUI(void const * argument);
void taskKey(void const * argument);
void taskChassis(void const * argument);
void taskHolder(void const * argument);
void taskJudge(void const * argument);
void taskShoot(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of RDtHMsg */
  osMessageQDef(RDtHMsg, 16, uint32_t);
  RDtHMsgHandle = osMessageCreate(osMessageQ(RDtHMsg), NULL);

  /* definition and creation of RDtCMsg */
  osMessageQDef(RDtCMsg, 16, uint32_t);
  RDtCMsgHandle = osMessageCreate(osMessageQ(RDtCMsg), NULL);

  /* definition and creation of RDtSMsg */
  osMessageQDef(RDtSMsg, 16, uint32_t);
  RDtSMsgHandle = osMessageCreate(osMessageQ(RDtSMsg), NULL);

  /* definition and creation of HDtHMsg */
  osMessageQDef(HDtHMsg, 16, uint32_t);
  HDtHMsgHandle = osMessageCreate(osMessageQ(HDtHMsg), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED */
  osThreadDef(LED, taskLED, osPriorityIdle, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of MSG */
  osThreadDef(MSG, taskMsg, osPriorityHigh, 0, 128);
  MSGHandle = osThreadCreate(osThread(MSG), NULL);

  /* definition and creation of UI */
  osThreadDef(UI, taskUI, osPriorityBelowNormal, 0, 128);
  UIHandle = osThreadCreate(osThread(UI), NULL);

  /* definition and creation of KEY */
  osThreadDef(KEY, taskKey, osPriorityBelowNormal, 0, 128);
  KEYHandle = osThreadCreate(osThread(KEY), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, taskChassis, osPriorityRealtime, 0, 512);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Holder */
  osThreadDef(Holder, taskHolder, osPriorityRealtime, 0, 512);
  HolderHandle = osThreadCreate(osThread(Holder), NULL);

  /* definition and creation of Judge */
  osThreadDef(Judge, taskJudge, osPriorityHigh, 0, 1024);
  JudgeHandle = osThreadCreate(osThread(Judge), NULL);

  /* definition and creation of Shoot */
  osThreadDef(Shoot, taskShoot, osPriorityAboveNormal, 0, 512);
  ShootHandle = osThreadCreate(osThread(Shoot), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_taskLED */
/**
  * @brief  Function implementing the LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_taskLED */
void taskLED(void const * argument)
{
  /* USER CODE BEGIN taskLED */
    HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LED4_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOD, LED1_Pin);
    HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
    HAL_GPIO_TogglePin(GPIOC, LED3_Pin);
    HAL_GPIO_TogglePin(GPIOC, LED4_Pin);  
    TIM1_SetPWMPluse(pluse);
		Vision_SendData();
    osDelay(1000);
  }
  /* USER CODE END taskLED */
}

/* USER CODE BEGIN Header_taskMsg */
/**
* @brief Function implementing the MSG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskMsg */
void taskMsg(void const * argument)
{
  /* USER CODE BEGIN taskMsg */
	keyboard_controlinit();
  /* Infinite loop */
  for(;;)
  {
    RemoteData_t RDMsg;
    HolderData_t HDMsg;
    RemoteDataMsg_Process(&RDMsg);
    HolderDataMsg_Process(&HDMsg);
	  keyboard_state_judge(RDMsg,HDMsg);
    osMessagePut(RDtHMsgHandle,(uint32_t)&RDMsg,0);
    osMessagePut(RDtCMsgHandle,(uint32_t)&RDMsg,0);
    osMessagePut(HDtHMsgHandle,(uint32_t)&HDMsg,0);
		osMessagePut(RDtSMsgHandle,(uint32_t)&HDMsg,0);
    osDelay(10);
  }
  /* USER CODE END taskMsg */
}

/* USER CODE BEGIN Header_taskUI */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskUI */
void taskUI(void const * argument)
{
  /* USER CODE BEGIN taskUI */
  UI_Init();
  /* Infinite loop */
  for(;;)
  {
    UI_Process();
    osDelay(10);
  }
  /* USER CODE END taskUI */
}

/* USER CODE BEGIN Header_taskKey */
/**
* @brief Function implementing the KEY thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskKey */
void taskKey(void const * argument)
{
  /* USER CODE BEGIN taskKey */
  /* Infinite loop */
  for(;;)
  {
    Key_Process();
    osDelay(10);
  }
  /* USER CODE END taskKey */
}

/* USER CODE BEGIN Header_taskChassis */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not use\d
* @retval None
*/
/* USER CODE END Header_taskChassis */
void taskChassis(void const * argument)
{
  /* USER CODE BEGIN taskChassis */
  Chassis_PidInit();
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(RDtCMsgHandle,0);
		evt1= osMessageGet(RDtSMsgHandle,0);
    if(evt.status == osEventMessage)
    {
        Observer.Rx.ChassisMsg_Rate++;
        RemoteData_t *RDMsg = (RemoteData_t*)evt.value.v;
			  HolderData_t *HDMsg = (HolderData_t*)evt1.value.v;
  		  Chassis_Process(*RDMsg,*HDMsg);
  	    
//				_TarCurrent=Chassis.M3508[0].TarCurrent;
//				_RxCurrent=Chassis.M3508[0].Rx.Current;
//				_TarSpeed=Chassis.M3508[0].TarSpeed;
//				_RxSpeed=Chassis.M3508[0].Rx.Speed;
//				_OutputLpf=Chassis.M3508[0].OutputLpf;
				
		}
		    osDelay(1);
  }
  /* USER CODE END taskChassis */
}

/* USER CODE BEGIN Header_taskHolder */
/*
* @brief Function implementing the Holder thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_taskHolder */
void taskHolder(void const * argument)
{
  /* USER CODE BEGIN taskHolder */
	osDelay(1000);
	Holder_PidInit();
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(RDtHMsgHandle,0);
		evt1= osMessageGet(HDtHMsgHandle,0);
    if(evt.status == osEventMessage)
    {
        Observer.Rx.HolderMsg_Rate++;
        RemoteData_t *RDMsg = (RemoteData_t*)evt.value.v;
			  HolderData_t *HDMsg = (HolderData_t*)evt1.value.v;
			  HolderMsg_Process(*RDMsg,*HDMsg);
			UART_SendWave(2,2,&Chassis.MoveData.Front,&Chassis.MoveData.Right);
//      UART_SendWave(5,2,&Holder.Pitch._0x20A.TarAngleLpf,&Holder.Pitch._0x20A.Rx.AngleLpf,&Holder.Pitch._0x20A.TarSpeed,&Holder.Pitch.gyoSpeed,&Holder.Pitch._0x20A.OutputLpf);
//		    UART_SendWave(5,2,&Holder.Yaw._0x209.TarAngleLpf,&Holder.Yaw.Rx.Angle,&Holder.Yaw._0x209.TarSpeed,&Holder.Yaw.Rx.Speed,&Holder.Yaw._0x209.OutputLpf);
//			UART_SendWave(4,2,&Holder.Yaw._0x209.TarAngleLpf,&Holder.Yaw.Rx.Angle,&Holder.Pitch._0x20A.TarAngleLpf,&Holder.Pitch._0x20A.Rx.Angle); 
		}
		osDelay(1);
	}
  /* USER CODE END taskHolder */
}

/* USER CODE BEGIN Header_taskJudge */
/**
* @brief Function implementing the Judge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskJudge */
void taskJudge(void const * argument)
{
  /* USER CODE BEGIN taskJudge */
	Judge_InitData();
  /* Infinite loop */
  for(;;)
  {
     Judge_Process();
     osDelay(10);
  }
  /* USER CODE END taskJudge */
}

/* USER CODE BEGIN Header_taskShoot */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskShoot */
void taskShoot(void const * argument)
{
  /* USER CODE BEGIN taskShoot */
	Ammunition_PidInit();
	Frictiongear_PidInit();	
  /* Infinite loop */
  for(;;)
  {
		Ammunition_Process();  
//		UART_SendWave(2,2,&Ammunition.RxSpeed, &Ammunition.TarSpeed);
//		Frictiongear_Process();
//    UART_SendWave(5,2,&Frictiongear.Rxspeed,&Frictiongear._0x202.TarSpeed,&Frictiongear._0x201.Rx.SpeedLpf,&Frictiongear._0x201.TarSpeed,&Frictiongear._0x202.OutputLpf);
//  	UART_SendWave(7,2,&Ammunition.TarCurrent,&Ammunition.RxCurrentLPf,&Ammunition.TarSpeed,&Ammunition.RxSpeedLPf,&Ammunition._0x206.OutputLpf,&Frictiongear._0x201.Rx.SpeedLpf,&Frictiongear.Rxspeed);
		
		osDelay(10);
  }
  /* USER CODE END taskShoot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
