/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v30x_misc.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file contains all the functions prototypes for the 
*                      miscellaneous firmware library functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/  
#ifndef __CH32V30X_MISC_H
#define __CH32V30X_MISC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v30x.h"

/* NVIC Init Structure definition */	 
typedef struct
{
  uint8_t NVIC_IRQChannel;
  uint8_t NVIC_IRQChannelPreemptionPriority;
  uint8_t NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} nvic_init_t;
 

/* Preemption_Priority_Group */
#define NVIC_PriorityGroup_0           ((uint32_t)0x00)
#define NVIC_PriorityGroup_1           ((uint32_t)0x01)
#define NVIC_PriorityGroup_2           ((uint32_t)0x02)
#define NVIC_PriorityGroup_3           ((uint32_t)0x03)
#define NVIC_PriorityGroup_4           ((uint32_t)0x04)


void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(nvic_init_t* NVIC_InitStruct);

#ifdef __cplusplus
}
#endif

#endif 

