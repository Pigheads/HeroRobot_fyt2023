#ifndef _JUDGE_H
#define _JUDGE_H

#include "judge_rx.h"
#include "dma_unpack.h"
#include "usart.h"

void Judge_InitData(void);
void Judge_Process(void);

/* Ħ���ֵȼ� */
extern uint8_t JudgeUI_rate_level;

/* С����״̬ */
extern uint8_t JudgeUI_spin_state;
#endif
