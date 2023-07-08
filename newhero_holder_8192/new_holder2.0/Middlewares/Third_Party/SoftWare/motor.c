/**
  ******************************************************************************
  * @file    
  * @author  sy
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
#include "motor.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/
#define CIRCLE_FASTEST 1000
void Circle_Continue(Mcircle_t *Mc, uint16_t angle)
{
    if ((angle < CIRCLE_FASTEST) && (Mc->Angle > 8192 - CIRCLE_FASTEST))
    {
        Mc->Circle++;
    }
    else if ((angle > 8192 - CIRCLE_FASTEST) && (Mc->Angle < CIRCLE_FASTEST))
    {
        Mc->Circle--;
    }
    Mc->Angle = angle;
}



/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
