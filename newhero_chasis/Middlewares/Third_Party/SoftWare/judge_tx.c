/**
  ******************************************************************************
  * @file    
  * @author  v5.0hxl
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
#include "judge_tx.h"
#include "dma_unpack.h"
#include "chassis.h"
#include "holder.h"
#include "string.h"
#include "vision.h"
#include "mathfun.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
uint8_t StrTmp[30] = {0}; /*�ַ����������*/
ext_client_graphic_drawtwo_t ext_client_graphic_drawtwo;
ext_client_graphic_drawseven_t ext_client_graphic_drawseven;
ext_client_graphic_drawtext_t judge_txmsg;
/* function ------------------------------------------------------------------*/
/*
    �������,��ʾ������,�ݱ���
	  ext_client_graphic_draw.ext_client_graphic_draw.radius = (numberint)&0x3FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_x = (numberint>>10)&0x7FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_y = (numberint>>21)&0x3FF;
*/
/**
  * @brief  ��ö�Ӧ���տͻ���ID
  * @param  uint16_t sender_ID
  * @retval uint16_t client_ID
  * @attention 
  */
static uint16_t get_client_ID(uint16_t sender_ID)
{
    uint16_t client_ID;
    switch (sender_ID)
    {
    case RED1_robotID:
        client_ID = 0x0101;
        break;
    case RED2_robotID:
        client_ID = 0x0102;
        break;
    case RED3_robotID:
        client_ID = 0x0103;
        break;
    case RED4_robotID:
        client_ID = 0x0104;
        break;
    case RED5_robotID:
        client_ID = 0x0105;
        break;
    case RED6_robotID:
        client_ID = 0x0106;
        break;

    case BLUE1_robotID:
        client_ID = 0x0165;
        break;
    case BLUE2_robotID:
        client_ID = 0x0166;
        break;
    case BLUE3_robotID:
        client_ID = 0x0167;
        break;
    case BLUE4_robotID:
        client_ID = 0x0168;
        break;
    case BLUE5_robotID:
        client_ID = 0x0169;
        break;
    case BLUE6_robotID:
        client_ID = 0x016A;
        break;
    }
    return client_ID;
}

/**
  * @brief  ����Ħ��������
  * @param  uint16_t sender_ID���Ͷ�ID
  * @param  uint8_t rate_levelĦ�������ٵȼ�
  * @param  uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendRateLv(uint16_t sender_ID, uint8_t rate_level, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '1';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 300;
    judge_txmsg.ext_client_graphic_draw.start_y = 650;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'P';
    judge_txmsg.text[1] = 'o';
    judge_txmsg.text[2] = 'w';
    judge_txmsg.text[3] = 'e';
		judge_txmsg.text[4] = 'r';
    judge_txmsg.text[5] = ':';
    judge_txmsg.text[6] = rate_level;
    
    if (rate_level == '0') /*Ħ����û����Ϊ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ���͵����˶�״̬
  * @param  uint16_t sender_ID���Ͷ�ID, 
  * @param  uint8_t spin_stateС����״̬,
  * @param  uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendSpinMode(uint16_t sender_ID, uint8_t spin_state, uint8_t operate_tpye)
{
    int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '2';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 300;
    judge_txmsg.ext_client_graphic_draw.start_y = 600;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'M';
    judge_txmsg.text[1] = 'o';
    judge_txmsg.text[2] = 'd';
    judge_txmsg.text[3] = 'e';
    judge_txmsg.text[4] = ':'; 
    judge_txmsg.text[5] = spin_state;

    if (spin_state == 'P') /*ͣ��ģʽΪ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 8;
		if (spin_state == 'V') /*С����ģʽΪ��ɫ*/
        judge_txmsg.ext_client_graphic_draw.color = 3;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ���ͳ�����������
  * @param  uint16_t sender_ID���Ͷ�ID,uint8_t *Str,float tmp,uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendSuperCapData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '3';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 1400;
    judge_txmsg.ext_client_graphic_draw.start_y = 700;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 1);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 1, StrTmp, sizeof(StrTmp));

    /*���ݷ�װ*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA���ͣ�����ַ���*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}

/**
  * @brief  ����Pitch����������
  * @param  uint16_t sender_ID���Ͷ�ID,uint8_t *Str,float tmp,uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendHolderPitchData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 800;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 1);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 1, StrTmp, sizeof(StrTmp));

//    /*���ݷ�װ*/
//    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
//    /*DMA���ͣ�����ַ���*/
//    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
//    memset(&StrTmp, 0, sizeof(StrTmp));
}

/**
  * @brief  ���͵���λ������ͼ�Ρ��������Լ��Ӵ���׼����������仯��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t len��������, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG2_SuperCap_Line(uint16_t sender_ID, uint16_t len, uint8_t operate_tpye)
{
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].operate_tpye = operate_tpye; //�仯����
	  ext_client_graphic_drawseven.ext_client_graphic_draw[5].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].operate_tpye = operate_tpye;
	
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 500;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 100;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 920;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 1000;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1420;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 100;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 1400;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 650;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1400+len;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 650;

    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 960+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 0;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 1397;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 656;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 1736;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 644;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ������׼Ŀ��ͼ�Σ�����仯��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */

void JudgeUI_SendBKG1(uint16_t sender_ID,uint8_t operate_tpye)
{
    int8_t i;
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    for (i = 0; i < 7; i++)
    {
        ext_client_graphic_drawseven.ext_client_graphic_draw[i].operate_tpye = operate_tpye;
    }

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 950+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 250;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 970+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 250;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 930+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 300;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 990+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 300;
		
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0; //10 m
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 900+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1020+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 350;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 880+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 390;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1040+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 390;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0; //5m
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 850+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 430;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1070+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 430;

    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 960+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 960+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 500;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 2;  //6m
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 960+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 430;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 20;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 0+10;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 0;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}
/**
  * @brief  ������׼Ŀ��ͼ��2������仯��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */

void JudgeUI_SendBKG2(uint16_t sender_ID,uint8_t operate_tpye)
{
    int8_t i;
    ext_client_graphic_drawtwo.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawtwo.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawtwo.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    for (i = 0; i < 2; i++)
    {
        ext_client_graphic_drawtwo.ext_client_graphic_draw[i].operate_tpye = operate_tpye;
    }

    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].graphic_name[0] = '2';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].layer = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].color = 6;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].width = 1;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].start_x = 910;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].start_y = 480;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].end_x = 910;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[0].end_y = 400;

    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].graphic_name[0] = '2';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].color = 6;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].width = 1;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].start_x = 1010+10;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].start_y = 480;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].end_x = 1010+10;
    ext_client_graphic_drawtwo.ext_client_graphic_draw[1].end_y = 400;
		
    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawtwo, sizeof(ext_client_graphic_drawtwo), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}
/**
  * @brief  ���͵�������̨�����λ��ͼ��
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */

void JudgeUI_SendChassisDis(uint16_t sender_ID,uint8_t operate_tpye)
{
    int8_t i;
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    for (i = 0; i < 7; i++)
    {
        ext_client_graphic_drawseven.ext_client_graphic_draw[i].operate_tpye = operate_tpye;
    }

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 1350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 335;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 1350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 435;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 1350-100*cos_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 335-100*sin_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 1350-100*cos_x(Holder.Yaw._0x209.Rx.Angle+60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 335-100*sin_x(Holder.Yaw._0x209.Rx.Angle+60);
		
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0; //10 m
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 1350-100*cos_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 335-100*sin_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1350+100*cos_x(Holder.Yaw._0x209.Rx.Angle+60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 335+100*sin_x(Holder.Yaw._0x209.Rx.Angle+60);

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 1350-100*cos_x(Holder.Yaw._0x209.Rx.Angle+60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 335-100*sin_x(Holder.Yaw._0x209.Rx.Angle+60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1350+100*cos_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 335+100*sin_x(Holder.Yaw._0x209.Rx.Angle-60);

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0; //5m
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 1350+100*cos_x(Holder.Yaw._0x209.Rx.Angle+60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 335+100*sin_x(Holder.Yaw._0x209.Rx.Angle+60);;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1350+100*cos_x(Holder.Yaw._0x209.Rx.Angle-60);
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 335+100*sin_x(Holder.Yaw._0x209.Rx.Angle-60);

    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 1345;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 335;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 1355;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 335;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 0;  //6m
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 1345;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 435;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 1355;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 435;

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
* @brief  ����ͼ3�ͳ���������������
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG3(uint8_t robot_id)
{
	static uint8_t tick = 0;
	if(tick < 10)
	{
		JudgeUI_SendChassisDis((uint16_t)(robot_id&0x00FF),2);
	}
	else
	{
		JudgeUI_SendChassisDis((uint16_t)(robot_id&0x00FF),1);
		if(tick == 10)
			tick = 0;
	}
	tick++;
}
/**
  * @brief  ������׼Ŀ��߶�
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendShootHight(uint16_t sender_ID,uint8_t high,uint8_t operate_tpye)
{
	  int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '1';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 15;
    judge_txmsg.ext_client_graphic_draw.end_angle = 5;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 1;
    judge_txmsg.ext_client_graphic_draw.width = 2;
	  judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
	
	  for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    
	 if(high==HIGHT_6M)
	 {
	  judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
    judge_txmsg.ext_client_graphic_draw.start_x = 1060;
    judge_txmsg.ext_client_graphic_draw.start_y = 455;
		judge_txmsg.text[0] = '5';
    judge_txmsg.text[1] = 'm';
	 }
   if(high==HIGHT_10M)
	 {
	  judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '5';
    judge_txmsg.ext_client_graphic_draw.start_x = 1010;
    judge_txmsg.ext_client_graphic_draw.start_y = 370;
		judge_txmsg.text[0] = '1';
		judge_txmsg.text[1] = '0';
    judge_txmsg.text[2] = 'm';
	 }
    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ���͵���״̬
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendShootBlock(uint16_t sender_ID,uint8_t state,uint8_t operate_tpye)
{
	  int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '1';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '6';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 700;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'B';
    judge_txmsg.text[1] = 'l';
    judge_txmsg.text[2] = 'o';
    judge_txmsg.text[3] = 'c';
    judge_txmsg.text[4] = 'k';
		judge_txmsg.text[5] = ':';
    if(state==0)
    {			
		 judge_txmsg.text[6] = 'N';
		}
		else
		{
		 judge_txmsg.text[7] = 'U';
		 judge_txmsg.text[8] = 'N';
		 judge_txmsg.ext_client_graphic_draw.color = 8;// ���ֿ���Ϊ��ɫ
		}

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  ������׼״̬
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendAimState(uint16_t sender_ID,uint8_t state,uint8_t operate_tpye)
{
	  int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '1';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '7';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 550;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
		
    judge_txmsg.text[0] = 'A';
    judge_txmsg.text[1] = 'I';
    judge_txmsg.text[2] = 'M';
    judge_txmsg.text[3] = ':';
    switch(state)
		{
			case 0:
         judge_txmsg.text[4] = 'M';
			break;
			case 1:
		     judge_txmsg.text[4] = 'S';
		     judge_txmsg.ext_client_graphic_draw.color = 8;// �̶���׼Ϊ��ɫ
      break;
			case 2:
         judge_txmsg.text[4] = 'A';
		     judge_txmsg.ext_client_graphic_draw.color = 3;// ����Ϊ��ɫ		
      break;
		}			

    /*���ݷ�װ*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA����*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  �����Ӿ�����߶�
  * @param  uint16_t sender_ID���Ͷ�ID, uint16_t client_ID���ն�ID��ʱ���ù�, uint8_t operate_tpye��ͼ��Ĳ���
  * @retval void
  * @attention 
  */
void JudgeUI_SendTargetDistance(uint16_t sender_ID, uint8_t *Str1,float hight, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '1';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '8';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 1500;
    judge_txmsg.ext_client_graphic_draw.start_y = 550;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
	  if(hight!=-1)
		{
      memcpy(judge_txmsg.text, Str1, sizeof(Str1) + 3);
      sprintf((char *)&StrTmp, "%4.2f", hight);
      memcpy(judge_txmsg.text + sizeof(Str1) + 3, StrTmp, sizeof(StrTmp));
			judge_txmsg.ext_client_graphic_draw.color = 3;
		}
		else
		{
			memcpy(judge_txmsg.text, Str1, sizeof(Str1)+3);
		}
    /*���ݷ�װ*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA���ͣ�����ַ���*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}

/**
* @brief  ����ͼ2�ͳ���������������
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_BK2_CapLine(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint16_t length;
	uint16_t CapValue = Chassis.SuperCap.cap_vol;
    length = (CapValue-15000)/30;
    if(CapValue < 15000)
        length = 0;
	if(tick < 3)
	{
		JudgeUI_SendBKG2_SuperCap_Line((uint16_t)(robot_id&0x00FF),length,2);
	}
	else
	{
		JudgeUI_SendBKG2_SuperCap_Line((uint16_t)(robot_id&0x00FF),length,1);
		if(tick == 3)
			tick = 0;
	}
	tick++;
}

/**
* @brief  ����������ֵ����
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_CapData(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint8_t Strc[30] = "Cap: ";
//	if(tick < 5)
//	{
		JudgeUI_SendSuperCapData((uint16_t)(robot_id&0x00FF),Strc,(float)0,2);
//	}
//	else
//	{
//		JudgeUI_SendSuperCapData((uint16_t)(robot_id&0x00FF),Strc,(float)0,1);
//		if(tick == 5)
//			tick = 0;
//	}
//	tick++;
}

/**
* @brief  �����Ƿ���
  * @param  void
  * @retval void
  * @attention 
  */
void JudgeUI_PitchData(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint8_t Strp[30] = "Pitch: ";
	if(tick < 5)
	{
		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)0,2);
	}
	else
	{
		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)0,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
* @brief  С����״̬���� 
  * @param  uint8_t state 'f':follow  'l':spin
  * @retval void
  * @attention 
  */
void JudgeUI_SpinMode(uint8_t robot_id, uint8_t state)
{
	static uint8_t tick = 0;
	if(tick < 5)
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  Ħ���ֵȼ�����
  * @param  uint8_t state 0/1/2/3
  * @retval void
  * @attention 
  */
void JudgeUI_ShootRateLv(uint8_t robot_id, uint8_t rate_lv)
{
	static uint8_t tick = 0;
    if(tick < 5)
	{
		JudgeUI_SendRateLv((uint16_t)(robot_id&0x00FF),rate_lv,2);
	}
	else
	{
		JudgeUI_SendRateLv((uint16_t)(robot_id&0x00FF),rate_lv,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  ��׼���뷢��
  * @param  6m 10m
  * @retval void
  * @attention 
  */
void JudgeUI_ShootHight(uint8_t robot_id,uint8_t high)
{
	static uint8_t tick = 0;
    if(tick < 5)
	{
		JudgeUI_SendShootHight((uint16_t)(robot_id&0x00FF),high,2);
	}
	else
	{
		JudgeUI_SendShootHight((uint16_t)(robot_id&0x00FF),high,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  ����״̬����
  * @param  uint8_t state 0/1
  * @retval void
  * @attention 
  */
void JudgeUI_ShootBlock(uint8_t robot_id,uint8_t state)
{
	static uint8_t tick = 0;
  if(tick < 5)
	{
		JudgeUI_SendShootBlock((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendShootBlock((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}


/**
  * @brief  ��׼״̬����
  * @param  uint8_t state 0/1/2
  * @retval void
  * @attention 
  */
void JudgeUI_AimState(uint8_t robot_id,uint8_t state)
{
	static uint8_t tick = 0;
  if(tick < 5)
	{
		JudgeUI_SendAimState((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendAimState((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  �Ӿ�����߶ȷ���
  * @param  uint8_t -1 hight
  * @retval void
  * @attention 
  */
void JudgeUI_TargetDistance(uint8_t robot_id)
{
	static uint8_t tick = 0;
	uint8_t Strc_1[] = "Target: ";
	if(tick < 5)
	{
		JudgeUI_SendTargetDistance((uint16_t)(robot_id&0x00FF),Strc_1,(float)vData.Pos.z,2);
	}
	else
	{
		JudgeUI_SendTargetDistance((uint16_t)(robot_id&0x00FF),Strc_1,(float)vData.Pos.z,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
