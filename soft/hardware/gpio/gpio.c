/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v30x_gpio.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the GPIO firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "rcc/rcc.h"

#include "gpio.h"

/* MASK */
#define ECR_PORTPINCONFIG_MASK    ((uint16_t)0xFF80)
#define LSB_MASK                  ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK      ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK        ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK      ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK       ((uint32_t)0x00100000)

/// \brief Deinitializes GPIO peripheral registers to their default value
/// \param gpiox - where x can be (A..G) to select the GPIO peripheral
/// \return None
void gpio_deinit(gpio_s *gpiox)
{
    if(gpiox == GPIOA)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
    }
    else if(gpiox == GPIOB)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);
    }
    else if(gpiox == GPIOC)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, DISABLE);
    }
    else if(gpiox == GPIOD)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, DISABLE);
    }
    else if(gpiox == GPIOE)
    {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, DISABLE);
    }
}

/// \brief Deinitializes the Alternate Functions (remap, event control
///        and EXTI configuration) registers to their default reset values.
/// \param None
/// \return None
void gpio_afio_deinit(void)
{
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
}

/// \brief Initialize GPIO
/// \param gpiox - where x can be (A..G) to select the GPIO peripheral
/// \param gpio_init_struct - pointer to a gpio_init_s structure that
///        contains the configuration information for the specified GPIO peripheral.
/// \return None
void gpio_init(gpio_s *gpiox, gpio_init_s *gpio_init_struct)
{
    uint32_t current_reg = 0;
    uint32_t gpio_pin_mask = 0xf;
    uint32_t gpio_pin_reg_pos = 0;
    uint32_t gpio_pin_new_mode = 0;

    for (uint32_t gpio_num = 0; gpio_num < GPIO_NUM; gpio_num++)
    {
        if (0 != gpio_init_struct->gpio_pins & (1 << gpio_num))
        {
            if (gpio_num <= (GPIO_NUM - 1) / 2)
            {
                current_reg = gpiox->CFGLR;
            }
            else
            {
                current_reg = gpiox->CFGHR;
            }
            gpio_pin_new_mode = (gpio_init_struct->gpio_mode & 0xf) | gpio_init_struct->gpio_speed;

            gpio_pin_reg_pos = gpio_num << 2;
            current_reg &= ~(gpio_pin_mask << gpio_pin_reg_pos);
            current_reg |= gpio_pin_new_mode << gpio_pin_reg_pos;

            if(gpio_init_struct->gpio_mode == GPIO_MODE_IPD)
            {
                gpiox->BCR = 1 << gpio_num;
            }
            else
            {
                if(gpio_init_struct->gpio_mode == GPIO_MODE_IPU)
                {
                    gpiox->BSHR = 1 << gpio_num;
                }
            }

            if (gpio_num <= (GPIO_NUM - 1) / 2)
            {
                gpiox->CFGLR = current_reg;
            }
            else
            {
                gpiox->CFGHR = current_reg;
            }
        }
    }
}

/// \brief Fill gpio_init_struct with default params for all GPIOs.
/// \param gpio_init_struct - pointer to a gpio_init_s structure that
///        contains the configuration information for the specified GPIO peripheral.
/// \return None
void gpio_struct_init_default(gpio_init_s *gpio_init_struct)
{
    gpio_init_struct->gpio_pins = GPIO_PIN_ALL;
    gpio_init_struct->gpio_speed = GPIO_SPEED_2MHZ;
    gpio_init_struct->gpio_mode = GPIO_MODE_IN_FLOATING;
}

/// \brief Read input data from gpio_pin
/// \param gpiox - where x can be (A..G) to select the GPIO peripheral
/// \param gpio_pin - gpio pin num
/// \return input status (0 or 1)
uint8_t gpio_read_input_data_bit(gpio_s *gpiox, uint16_t gpio_pin)
{
    return (gpiox->INDR & gpio_pin) != BIT_RESET;
}

/*********************************************************************
 * @fn      GPIO_ReadInputData
 *
 * @brief   Reads the specified GPIO input data port.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *
 * @return  The output port pin value.
 */
uint16_t gpio_read_input_data(gpio_s *gpiox)
{
    return ((uint16_t)gpiox->INDR);
}

/*********************************************************************
 * @fn      GPIO_ReadOutputDataBit
 *
 * @brief   Reads the specified output data port bit.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
uint8_t gpio_read_output_data_bit(gpio_s *gpiox, uint16_t gpio_pin)
{
    uint8_t bitstatus = 0x00;

    if((gpiox->OUTDR & gpio_pin) != (uint32_t)BIT_RESET)
    {
        bitstatus = (uint8_t)BIT_SET;
    }
    else
    {
        bitstatus = (uint8_t)BIT_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_ReadOutputData
 *
 * @brief   Reads the specified GPIO output data port.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *
 * @return  GPIO output port pin value.
 */
uint16_t gpio_read_output_data(gpio_s *gpiox)
{
    return ((uint16_t)gpiox->OUTDR);
}

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void gpio_set_bits(gpio_s *gpiox, uint16_t gpio_pin)
{
    gpiox->BSHR = gpio_pin;
}

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void gpio_reset_bits(gpio_s *gpiox, uint16_t gpio_pin)
{
    gpiox->BCR = gpio_pin;
}

/*********************************************************************
 * @fn      GPIO_WriteBit
 *
 * @brief   Sets or clears the selected data port bit.
 *
 * @param   GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be one of GPIO_Pin_x where x can be (0..15).
 *          BitVal - specifies the value to be written to the selected bit.
 *            Bit_RESET - to clear the port pin.
 *            Bit_SET - to set the port pin.
 *
 * @return  none
 */
void gpio_write_bit(gpio_s *gpiox, uint16_t gpio_pin, bit_action_e bit_val)
{
    if(bit_val != BIT_RESET)
    {
        gpiox->BSHR = gpio_pin;
    }
    else
    {
        gpiox->BCR = gpio_pin;
    }
}

/*********************************************************************
 * @fn      GPIO_Write
 *
 * @brief   Writes data to the specified GPIO data port.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *          PortVal - specifies the value to be written to the port output data register.
 *
 * @return  none
 */
void gpio_write(gpio_s *gpiox, uint16_t port_val)
{
    gpiox->OUTDR = port_val;
}

/*********************************************************************
 * @fn      GPIO_PinLockConfig
 *
 * @brief   Locks GPIO Pins configuration registers.
 *
 * @param   GPIOx - where x can be (A..G) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void gpio_pin_lock_config(gpio_s *gpiox, uint16_t gpio_pin)
{
    uint32_t tmp = 0x00010000;

    tmp |= gpio_pin;
    gpiox->LCKR = tmp;
    gpiox->LCKR = gpio_pin;
    gpiox->LCKR = tmp;
    tmp = gpiox->LCKR;
    tmp = gpiox->LCKR;
}

/*********************************************************************
 * @fn      GPIO_EventOutputConfig
 *
 * @brief   Selects the GPIO pin used as Event output.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source
 *        for Event output.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..E).
 *          GPIO_PinSource - specifies the pin for the Event output.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void gpio_event_output_config(uint8_t gpio_port_source, uint8_t gpio_pin_source)
{
    uint32_t tmpreg = 0x00;

    tmpreg = AFIO->ECR;
    tmpreg &= ECR_PORTPINCONFIG_MASK;
    tmpreg |= (uint32_t)gpio_port_source << 0x04;
    tmpreg |= gpio_pin_source;
    AFIO->ECR = tmpreg;
}

/*********************************************************************
 * @fn      GPIO_EventOutputCmd
 *
 * @brief   Enables or disables the Event Output.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void gpio_event_output_cmd(functional_state_e new_state)
{
    if(new_state)
    {
        AFIO->ECR |= (1 << 7);
    }
    else
    {
        AFIO->ECR &= ~(1 << 7);
    }
}

/*********************************************************************
 * @fn      GPIO_PinRemapConfig
 *
 * @brief   Changes the mapping of the specified pin.
 *
 * @param   GPIO_Remap - selects the pin to remap.
 *            GPIO_Remap_SPI1 - SPI1 Alternate Function mapping
 *            GPIO_Remap_I2C1 - I2C1 Alternate Function mapping
 *            GPIO_Remap_USART1 - USART1 Alternate Function mapping
 *            GPIO_Remap_USART2 - USART2 Alternate Function mapping
 *            GPIO_PartialRemap_USART3 - USART3 Partial Alternate Function mapping
 *            GPIO_PartialRemap1_USART3 - USART3 Partial1 Alternate Function mapping
 *            GPIO_FullRemap_USART3 - USART3 Full Alternate Function mapping
 *            GPIO_PartialRemap_TIM1 - TIM1 Partial Alternate Function mapping
 *            GPIO_FullRemap_TIM1 - TIM1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_TIM2 - TIM2 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_TIM2 - TIM2 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_TIM2 - TIM2 Full Alternate Function mapping
 *            GPIO_PartialRemap_TIM3 - TIM3 Partial Alternate Function mapping
 *            GPIO_FullRemap_TIM3 - TIM3 Full Alternate Function mapping
 *            GPIO_Remap_TIM4 - TIM4 Alternate Function mapping
 *            GPIO_Remap1_CAN1 - CAN1 Alternate Function mapping
 *            GPIO_Remap2_CAN1 - CAN1 Alternate Function mapping
 *            GPIO_Remap_PD01 - PD01 Alternate Function mapping
 *            GPIO_Remap_ADC1_ETRGINJ - ADC1 External Trigger Injected Conversion remapping
 *            GPIO_Remap_ADC1_ETRGREG - ADC1 External Trigger Regular Conversion remapping
 *            GPIO_Remap_ADC2_ETRGINJ - ADC2 External Trigger Injected Conversion remapping
 *            GPIO_Remap_ADC2_ETRGREG - ADC2 External Trigger Regular Conversion remapping
 *            GPIO_Remap_ETH - Ethernet remapping
 *            GPIO_Remap_CAN2 - CAN2 remapping
 *            GPIO_Remap_MII_RMII_SEL - MII or RMII selection
 *            GPIO_Remap_SWJ_NoJTRST - Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST
 *            GPIO_Remap_SWJ_JTAGDisable - JTAG-DP Disabled and SW-DP Enabled
 *            GPIO_Remap_SWJ_Disable - Full SWJ Disabled (JTAG-DP + SW-DP)
 *            GPIO_Remap_TIM2ITR1_PTP_SOF - Ethernet PTP output or USB OTG SOF (Start of Frame) connected
 *        to TIM2 Internal Trigger 1 for calibration
 *            GPIO_Remap_TIM2ITR1_PTP_SOF - Ethernet PTP output or USB OTG SOF (Start of Frame)
 *            GPIO_Remap_TIM8 - TIM8 Alternate Function mapping
 *            GPIO_PartialRemap_TIM9 - TIM9 Partial Alternate Function mapping
 *            GPIO_FullRemap_TIM9 - TIM9 Full Alternate Function mapping
 *            GPIO_PartialRemap_TIM10 - TIM10 Partial Alternate Function mapping
 *            GPIO_FullRemap_TIM10 - TIM10 Full Alternate Function mapping
 *            GPIO_Remap_FSMC_NADV - FSMC_NADV Alternate Function mapping
 *            GPIO_PartialRemap_USART4 - USART4 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART4 - USART4 Full Alternate Function mapping
 *            GPIO_PartialRemap_USART5 - USART5 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART5 - USART5 Full Alternate Function mapping
 *            GPIO_PartialRemap_USART6 - USART6 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART6 - USART6 Full Alternate Function mapping
 *            GPIO_PartialRemap_USART7 - USART7 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART7 - USART7 Full Alternate Function mapping
 *            GPIO_PartialRemap_USART8 - USART8 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART8 - USART8 Full Alternate Function mapping
 *            GPIO_Remap_USART1_HighBit - USART1 Alternate Function mapping high bit
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void gpio_pin_remap_config(uint32_t gpio_remap, functional_state_e new_state)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

    if((gpio_remap & 0x80000000) == 0x80000000)
    {
        tmpreg = AFIO->PCFR2;
    }
    else
    {
        tmpreg = AFIO->PCFR1;
    }

    tmpmask = (gpio_remap & DBGAFR_POSITION_MASK) >> 0x10;
    tmp = gpio_remap & LSB_MASK;

    /* Clear bit */
    if((gpio_remap & 0x80000000) == 0x80000000)
    {                                                                                                                   /* PCFR2 */
        if((gpio_remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) /* [31:16] 2bit */
        {
            tmp1 = ((uint32_t)0x03) << (tmpmask + 0x10);
            tmpreg &= ~tmp1;
        }
        else if((gpio_remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) /* [15:0] 2bit */
        {
            tmp1 = ((uint32_t)0x03) << tmpmask;
            tmpreg &= ~tmp1;
        }
        else /* [31:0] 1bit */
        {
            tmpreg &= ~(tmp << (((gpio_remap & 0x7FFFFFFF)>> 0x15) * 0x10));
        }
    }
    else
    {                                                                                                                   /* PCFR1 */
        if((gpio_remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) /* [26:24] 3bit SWD_JTAG */
        {
            tmpreg &= DBGAFR_SWJCFG_MASK;
            AFIO->PCFR1 &= DBGAFR_SWJCFG_MASK;
        }
        else if((gpio_remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) /* [15:0] 2bit */
        {
            tmp1 = ((uint32_t)0x03) << tmpmask;
            tmpreg &= ~tmp1;
            tmpreg |= ~DBGAFR_SWJCFG_MASK;
        }
        else /* [31:0] 1bit */
        {
            tmpreg &= ~(tmp << ((gpio_remap >> 0x15) * 0x10));
            tmpreg |= ~DBGAFR_SWJCFG_MASK;
        }
    }

    /* Set bit */
    if(new_state != DISABLE)
    {
        tmpreg |= (tmp << (((gpio_remap & 0x7FFFFFFF)>> 0x15) * 0x10));
    }

    if((gpio_remap & 0x80000000) == 0x80000000)
    {
        AFIO->PCFR2 = tmpreg;
    }
    else
    {
        AFIO->PCFR1 = tmpreg;
    }
}

/*********************************************************************
 * @fn      GPIO_EXTILineConfig
 *
 * @brief   Selects the GPIO pin used as EXTI Line.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source for EXTI lines.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..G).
 *          GPIO_PinSource - specifies the EXTI line to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void gpio_extiline_config(uint8_t gpio_port_source, uint8_t gpio_pin_source)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (gpio_pin_source & (uint8_t)0x03));
    AFIO->EXTICR[gpio_pin_source >> 0x02] &= ~tmp;
    AFIO->EXTICR[gpio_pin_source >> 0x02] |= (((uint32_t)gpio_port_source) << (0x04 * (gpio_pin_source & (uint8_t)0x03)));
}

/*********************************************************************
 * @fn      GPIO_ETH_MediaInterfaceConfig
 *
 * @brief   Selects the Ethernet media interface.
 *
 * @param   GPIO_ETH_MediaInterface - specifies the Media Interface mode.
 *            GPIO_ETH_MediaInterface_MII - MII mode
 *            GPIO_ETH_MediaInterface_RMII - RMII mode
 *
 * @return  none
 */
void gpio_eth_media_interface_config(uint32_t gpio_eth_media_interface)
{
    if(gpio_eth_media_interface)
    {
        AFIO->PCFR1 |= (1 << 23);
    }
    else
    {
        AFIO->PCFR1 &= ~(1 << 23);
    }
}
