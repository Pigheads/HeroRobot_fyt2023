#include "judge.h"
#include "judge_tx.h"
#include "judge_rx.h"
#include "usart.h"
#include "cmsis_os.h"
#include "keyboard.h"
#include "chassis.h"

#define JUDGE_FIFO_BUFLEN 500

uint8_t JudgeUI_rate_level;
uint8_t JudgeUI_spin_state;
uint8_t JudgeUI_block_state;
uint8_t JudgeUI_aim_state;

static unpack_data_t judge_unpack_obj;

static uart_dma_rx_t judge_rx_obj;
static fifo_s_t judge_rxdata_fifo;
static uint8_t judge_rxdata_buf[JUDGE_FIFO_BUFLEN];

void Judge_InitData(void)
{
    fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_FIFO_BUFLEN);

    judge_rx_obj.hdma_usart_rx = &hdma_usart3_rx;
    judge_rx_obj.p_fifo = &judge_rxdata_fifo;
    judge_rx_obj.buff_size = 1024;
    judge_rx_obj.buff[0] = uart3_m0_buf;
    judge_rx_obj.buff[1] = uart3_m1_buf;

    judge_unpack_obj.p_fifo = &judge_rxdata_fifo;
    judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
    judge_unpack_obj.index = 0;
    judge_unpack_obj.data_len = 0;
    judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
}

void UI_spin_state(void)
{
	if(Control_state.spin_move.cr_state==1)
		JudgeUI_spin_state='V';
	else
	{
		switch(Control_state.spin_state.cr_state)
		{
			case 0:
				JudgeUI_spin_state='P';
			break;
			case 1:
				JudgeUI_spin_state='F';
			break;
		}
	}
}

void UI_rate_level(void)
{
		switch(super_state)
		 {					
			 case 0:
          JudgeUI_rate_level='C';
			 break;
			 case 1: 
				 JudgeUI_rate_level='N';
			break;
		}	
}

void UI_block_state(void)
{
		 JudgeUI_block_state=Control_state.ShootBlock;
}

void UI_aim_state(void)
{
	if(Control_state.vision_state==0)
	{
	   if(Control_state.aim_state.cr_state==1)
			 JudgeUI_aim_state=1;
		 else
			 JudgeUI_aim_state=0;
	}
	else
		   JudgeUI_aim_state=2;
}

static void Judge_Uart_Idle(void)
{
    dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
    judge_unpack_fifo_data(&judge_unpack_obj, SOF);    
}

static void Judge_Uart_UI(void)
{
    static judge_txpoll_e judge_txpoll = _Rate_Lv;
    uint8_t robot_id = JUDGE_u8GetRobotId();
    switch(judge_txpoll)
    {
			case _Rate_Lv:					//等级
            JudgeUI_ShootRateLv((uint16_t)(robot_id&0x00FF),JudgeUI_rate_level);
			osDelay(5);
            judge_txpoll++;
            break;
        case _Spin_Mode:			//小陀螺状态
            JudgeUI_SpinMode((uint16_t)(robot_id&0x00FF),JudgeUI_spin_state);
			osDelay(5);
            judge_txpoll++;
            break;
        case _SuperCap_Data:	//超级电容
//            JudgeUI_CapData((uint16_t)(robot_id&0x00FF));
			osDelay(5);
            judge_txpoll++;
            break;
        case _HolderPitch_Data://俯仰角
//            JudgeUI_PitchData((uint16_t)(robot_id&0x00FF));
			osDelay(5);
            judge_txpoll++;
            break;
        case _BackGround1:		//瞄准目标图形 横线和圆
            JudgeUI_SendBKG1((uint16_t)(robot_id&0x00FF),1);
			osDelay(5);
            judge_txpoll++;
            break;
        case _BackGround2_SuperCap_Line://背景图2和超级电容线条
            JudgeUI_BK2_CapLine((uint16_t)(robot_id&0x00FF)); 
			osDelay(5);
            judge_txpoll++;
            break;
        case _Shoot_Hight_6m:		//瞄准距离6m
					  JudgeUI_ShootHight((uint16_t)(robot_id&0x00FF),HIGHT_6M);   
      osDelay(5);			
            judge_txpoll++;
            break;		
        case _Shoot_Hight_10m:  //瞄准距离10m
					  JudgeUI_ShootHight((uint16_t)(robot_id&0x00FF),HIGHT_10M);   
      osDelay(5);			
            judge_txpoll++;
//				judge_txpoll=_Rate_Lv;
            break;
				case _BackGround2:				//瞄准目标图形 竖线
				    JudgeUI_SendBKG2((uint16_t)(robot_id&0x00FF),1);
			osDelay(5);
				    judge_txpoll++;
//				judge_txpoll=_Rate_Lv;
            break;				
				case _ChassisDis:
					JudgeUI_SendBKG3((uint16_t)(robot_id&0x00FF));
				osDelay(5);
					judge_txpoll=_Rate_Lv;
					break;
//        case _Aim_State:				//瞄准状态
//				    JudgeUI_AimState((uint16_t)(robot_id&0x00FF),JudgeUI_aim_state);
//			osDelay(5);
//				    judge_txpoll++;
//            break;				
//			  case _Shoot_Bloclk:			//弹舱状态
//						JudgeUI_ShootBlock((uint16_t)(robot_id&0x00FF),JudgeUI_block_state);		
//      osDelay(5);
//				    judge_txpoll=_Rate_Lv;
//            break;
//        case _Tar_Distance:			//视觉相机高度
//            JudgeUI_TargetDistance((uint16_t)(robot_id&0x00FF));
//			osDelay(5);
// 				    judge_txpoll++; 				
//				    break;
    }
		UI_rate_level();
		UI_spin_state();
		UI_block_state();
		UI_aim_state();
}
int id_test=0;
void Judge_Process(void)
{
    if(uart3_idle_flag == 1)
    {
        uart3_idle_flag = 0;
        Judge_Uart_Idle();
    }
		uint8_t robot_id = JUDGE_u8GetRobotId();
		id_test = robot_id;
    if(robot_id != 0)
    {
        Judge_Uart_UI();
    }
}


