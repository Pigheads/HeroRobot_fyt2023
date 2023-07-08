/**
  ******************************************************************************
  * @file    
  * @author  sy
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
#ifndef _UI_LIB_H
#define _UI_LIB_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "oled.h"
#include "key.h"
/* define --------------------------------------------------------------------*/
#define PAGE_MAX  15
#define FUNC_MAX  10
#define PARAM_MAX 10

#define uiPageParamDef(page,x,str,param,paramtype)      \
        page.PageStr[x] = (uint8_t *)str;               \
        page.Param[x] = (void *)&param;                 \
        page.ParamType[x] = paramtype
        
#define uiPageFuncDef(page,x,str,func)                  \
        page.PageStr[x] = (uint8_t *)str;               \
        page.Func[x] = func;            
            
#define uiPageShowDef(page,show)                        \
        page.Show = show;           
            
#define uiPageDef(page,type,len)                        \
        page.PageLen = len;                             \
        page.PageType = type;           
                    
#define uiPageMainDef(page,type,len)                    \
        page.PageLen = len;                             \
        page.PageType = type;                           \
        page.IsMainMenu = 1;
        
        

/* typedef -------------------------------------------------------------------*/
typedef enum _Page_Type
{
    ChangePara = 1,
    Function,
    ShowStr    
}Page_Type_e;

typedef enum _Data_Type
{
	UINT32,
	INT32,
	UINT16,
	INT16,
	UINT8,
	INT8,
	FLOAT,    
}Data_Type_e;

typedef struct _Page_t
{
    uint8_t     PageLen;
    uint8_t     IsMainMenu;
    void        (*Show) (void);
    uint8_t     *PageStr[PAGE_MAX];
    void        (*Func[FUNC_MAX]) (void);
    void        *Param[PARAM_MAX];
    Data_Type_e ParamType[PARAM_MAX];
    Page_Type_e PageType;
}Page_t;

/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
void UI_Show(Page_t *page);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
