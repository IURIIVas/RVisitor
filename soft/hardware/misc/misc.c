/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_misc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the miscellaneous firmware functions .
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <misc.h>

__RW uint32_t nvic_priority_group = 0;

/*********************************************************************
 * @fn      NVIC_PriorityGroupConfig
 *
 * @brief   Configures the priority grouping - pre-emption priority and subpriority.
 *
 * @param   NVIC_PriorityGroup - specifies the priority grouping bits length.
 *            NVIC_PriorityGroup_0 - 0 bits for pre-emption priority
 *                                   4 bits for subpriority
 *            NVIC_PriorityGroup_1 - 1 bits for pre-emption priority
 *                                   3 bits for subpriority
 *            NVIC_PriorityGroup_2 - 2 bits for pre-emption priority
 *                                   2 bits for subpriority
 *            NVIC_PriorityGroup_3 - 3 bits for pre-emption priority
 *                                   1 bits for subpriority
 *            NVIC_PriorityGroup_4 - 4 bits for pre-emption priority
 *                                   0 bits for subpriority
 *
 * @return  none
 */
void nvic_priority_group_config(uint32_t NVIC_PriorityGroup)
{
    nvic_priority_group = NVIC_PriorityGroup;
}

/*********************************************************************
 * @fn      NVIC_Init
 *
 * @brief   Initializes the NVIC peripheral according to the specified parameters in
 *        the NVIC_InitStruct.
 *
 * @param   NVIC_InitStruct - pointer to a NVIC_InitTypeDef structure that contains the
 *        configuration information for the specified NVIC peripheral.
 *
 * @return  none
 */
void NVIC_Init(nvic_init_s *NVIC_InitStruct)
{
    uint8_t tmppre = 0;

    if(nvic_priority_group == NVIC_PriorityGroup_0)
    {
        __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4);
    }
    else if(nvic_priority_group == NVIC_PriorityGroup_1)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority == 1)
        {
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4));
        }
        else
        {
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4));
        }
    }
    else if(nvic_priority_group == NVIC_PriorityGroup_2)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority <= 1)
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (4 * NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority);
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (tmppre << 4));
        }
        else
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (4 * (NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority - 2));
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (tmppre << 4));
        }
    }
    else if(nvic_priority_group == NVIC_PriorityGroup_3)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority <= 3)
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (2 * NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority);
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (tmppre << 4));
        }
        else
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (2 * (NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority - 4));
            __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (tmppre << 4));
        }
    }
    else if(nvic_priority_group == NVIC_PriorityGroup_4)
    {
        __pfic_set_priority(NVIC_InitStruct->NVIC_IRQChannel, NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << 4);
    }

    if(NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
    {
        __pfic_enable_irq(NVIC_InitStruct->NVIC_IRQChannel);
    }
    else
    {
        __pfic_disable_irq(NVIC_InitStruct->NVIC_IRQChannel);
    }
}
