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
#include <stdbool.h>
#include "ui_lib.h"
#include "cmsis_os.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
KeyMsg_t UI_Key;

bool IsUIPageMain = true;
int8_t Cursor = 0;
int8_t Cursor_last = 0;
int8_t NowPage = 0;
int8_t Menustatus = 0;
uint8_t uintForNow = 0;
float DataInterval = 0;
float uintArray[5] = {100, 10, 1, 0.1, 0};
int8_t DataToChangeNum = 0;
/* function ------------------------------------------------------------------*/
void Key_Event(void)
{
    if(GetKeyMsgFromFifo(&UI_Key))
    {
        switch (UI_Key.Key)
        {
        case Up:
            if (!Menustatus) {
                if (UI_Key.Status == 0) {
                    Cursor--;
                }
            }
            else {
                if (UI_Key.Status == 0) {
                    DataInterval++;
                }
            }
            break;
        case Down:
            if (!Menustatus) {
                if (UI_Key.Status == 0) {
                    Cursor++;
                }
            }
            else {
                if (UI_Key.Status == 0) {
                    DataInterval--;
                }
            }
            break;
        case Enter:
            if (UI_Key.Status == 0) {
                if (!Menustatus)
                    NowPage++;
                else {
                    uintForNow++;
                    if (uintForNow == 5)
                        uintForNow = 0;
                }
            }
            break;
        case Back:
            if (UI_Key.Status == 0) {
                NowPage--;
                if (Menustatus == 1)
                    Menustatus = 0;
            }
            break;
        default:
            break;
        }
    }
}

void UI_ChangeParam(Page_t *page, uint8_t num, uint8_t LastPage, void *DataToChange[])
{
    uint32_t *data_tmp_u32;
    int32_t  *data_tmp_s32;
    uint16_t *data_tmp_u16;
    int16_t  *data_tmp_s16;
    uint8_t  *data_tmp_u8;
    int8_t   *data_tmp_s8;
    float    *data_tmp_f32;
    switch(page->ParamType[num])
    {
    case UINT32:
        data_tmp_u32 = (uint32_t *)page->Param[num];
        break;
    case INT32:
        data_tmp_s32 = (int32_t *)page->Param[num];
        break;
    case UINT16:
        data_tmp_u16 = (uint16_t *)page->Param[num];
        break;
    case INT16:
        data_tmp_s16 = (int16_t *)page->Param[num];
        break;
    case UINT8:
        data_tmp_u8 = (uint8_t *)page->Param[num];
        break;
    case INT8:
        data_tmp_s8 = (int8_t *)page->Param[num];
        break;
    case FLOAT:
        data_tmp_f32 = (float *)page->Param[num];
        break;
    }
    uint8_t *Object = page->PageStr[num];
    Menustatus = 1;
    uintForNow = 1;
    OLED_Clear();
    OLED_ShowString(0,0,Object);
    while (1)
    {
        Key_Event();
        if (NowPage == LastPage)
        {
            Menustatus = 0;
            OLED_Clear();
            break;
        }
        if (uintArray[uintForNow] != 0 )
        {
            switch(page->ParamType[num])
            {
            case UINT32:
                *data_tmp_u32 += DataInterval * uintArray[uintForNow];
                break;
            case INT32:
                *data_tmp_s32 += DataInterval * uintArray[uintForNow];
                break;
            case UINT16:
                *data_tmp_u16 += DataInterval * uintArray[uintForNow];
                break;
            case INT16:
                *data_tmp_s16 += DataInterval * uintArray[uintForNow];
                break;
            case UINT8:
                *data_tmp_u8 += DataInterval * uintArray[uintForNow];
                break;
            case INT8:
                *data_tmp_s8 += DataInterval * uintArray[uintForNow];
                break;
            case FLOAT:
                *data_tmp_f32 += DataInterval * uintArray[uintForNow];
                break;
            }
        }
        else if (uintArray[uintForNow] == 0 && DataInterval != 0)
        {
            switch(page->ParamType[num])
            {
            case UINT32:
                *data_tmp_u32 = 0;
                break;
            case INT32:
                *data_tmp_s32 = 0;
                break;
            case UINT16:
                *data_tmp_u16 = 0;
                break;
            case INT16:
                *data_tmp_s16 = 0;
                break;
            case UINT8:
                *data_tmp_u8 = 0;
                break;
            case INT8:
                *data_tmp_s8 = 0;
                break;
            case FLOAT:
                *data_tmp_f32 = 0;
                break;
            }
        }
        DataInterval = 0;
        switch(page->ParamType[num])
        {
        case UINT32:
            OLED_ShowFloat(0, 1, *data_tmp_u32,2);
            break;
        case INT32:
            OLED_ShowFloat(0, 1, *data_tmp_s32,2);
            break;
        case UINT16:
            OLED_ShowFloat(0, 1, *data_tmp_u16,2);
            break;
        case INT16:
            OLED_ShowFloat(0, 1, *data_tmp_s16,2);
            break;
        case UINT8:
            OLED_ShowFloat(0, 1, *data_tmp_u8,2);
            break;
        case INT8:
            OLED_ShowFloat(0, 1, *data_tmp_s8,2);
            break;
        case FLOAT:
            OLED_ShowFloat(0, 1, *data_tmp_f32,2);
            break;
        }
        if (uintArray[uintForNow] == 0)
        {
            OLED_ShowString(0, 2, (uint8_t*)"Clear ");
        }
        else
        {
            OLED_ShowFloat(0, 2, uintArray[uintForNow],2);
        }
        osDelay(10);
    }
}

void UI_ShowString(Page_t *page,uint8_t cursor)
{
    uint8_t y;
    uint8_t **pagestr = page->PageStr;
    for (y = 0; y < 8; y++)
    {
        if (y == page->PageLen)
            break;
        if (Cursor < 8)
        {
            if (y == Cursor)
            {
                OLED_ShowChar(0,y,123);
                OLED_ShowString(8, y, *(pagestr + y));
            }
            else
            {
                OLED_ShowChar(0,y,' ');
                OLED_ShowString(8, y, *(pagestr + y));
            }
            if (page->PageType == ChangePara)
            {
                switch(page->ParamType[y])
                {
                case UINT32:
                    OLED_ShowFloat(80, y, *((uint32_t *)(page->Param[y])),2);
                    break;
                case INT32:
                    OLED_ShowFloat(80, y, *((int32_t *)(page->Param[y])),2);
                    break;
                case UINT16:
                    OLED_ShowFloat(80, y, *((uint16_t *)(page->Param[y])),2);
                    break;
                case INT16:
                    OLED_ShowFloat(80, y, *((int16_t *)(page->Param[y])),2);
                    break;
                case UINT8:
                    OLED_ShowFloat(80, y, *((uint8_t *)(page->Param[y])),2);
                    break;
                case INT8:
                    OLED_ShowFloat(80, y, *((int8_t *)(page->Param[y])),2);
                    break;
                case FLOAT:
                    OLED_ShowFloat(80, y, *((float *)(page->Param[y])),2);
                    break;
                }
            }
        }
        else
        {
            if (y == 7)
            {
                OLED_ShowChar(0,y,123);
                OLED_ShowString(8, y, *(pagestr + Cursor - 7 + y));
            }
            else
            {
                OLED_ShowChar(0,y,' ');
                OLED_ShowString(8, y, *(pagestr + Cursor - 7 + y));
            }
            if (page->PageType == ChangePara)
            {                
                switch(page->ParamType[y])
                {
                case UINT32:
                    OLED_ShowFloat(80, y, *((uint32_t *)(page->Param[y])),2);
                    break;
                case INT32:
                    OLED_ShowFloat(80, y, *((int32_t *)(page->Param[y])),2);
                    break;
                case UINT16:
                    OLED_ShowFloat(80, y, *((uint16_t *)(page->Param[y])),2);
                    break;
                case INT16:
                    OLED_ShowFloat(80, y, *((int16_t *)(page->Param[y])),2);
                    break;
                case UINT8:
                    OLED_ShowFloat(80, y, *((uint8_t *)(page->Param[y])),2);
                    break;
                case INT8:
                    OLED_ShowFloat(80, y, *((int8_t *)(page->Param[y])),2);
                    break;
                case FLOAT:
                    OLED_ShowFloat(80, y, *((float *)(page->Param[y])),2);
                    break;
                }
            }
        }
    }
}

void UI_Show(Page_t *page)
{
    NowPage = 2;
    Cursor = 0;
    Menustatus = 0;
    OLED_Clear();
    while (1)
    {
        Key_Event();
        if (Cursor == page->PageLen)
            Cursor = 0;
        if (Cursor < 0 )
            Cursor = page->PageLen - 1;

        if (NowPage == 1)
        {
            if (page->IsMainMenu != 1)
            {
                OLED_Clear();
                break;
            } else {
                NowPage = 2;
            }
        }
        else if (NowPage == 3)
        {
            if (page->PageType == ChangePara)
                UI_ChangeParam(page, Cursor, 2, page->Param);
            else if(page->PageType == ShowStr)
            {
                NowPage = 2;
                page->Show();
            } else
            {
                NowPage = 2;
                Cursor_last = Cursor;
                OLED_Clear();
                page->Func[Cursor]();
                Cursor = Cursor_last;
            }
        }
        if (page->PageType != ShowStr)
            UI_ShowString(page,Cursor);
        else
            page->Show();

        if (page->IsMainMenu == 1)
        {
            IsUIPageMain = true;
        }
        else
        {
            IsUIPageMain = false;
        }
        osDelay(10);
    }
}


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
