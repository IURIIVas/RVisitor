/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v30x_gpio.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file contains all the functions prototypes for the 
*                      GPIO firmware library.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#ifndef __CH32V30x_GPIO_H
#define __CH32V30x_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v30x.h"
                                 
/* Output Maximum frequency selection */
typedef enum
{ 
    GPIO_INPUT_MODE = 0,
    GPIO_SPEED_10MHZ = 1,
    GPIO_SPEED_2MHZ = 2,
    GPIO_SPEED_50MHZ = 3
} gpio_speed_e;

/* Configuration Mode enumeration */
typedef enum
{
    GPIO_MODE_AIN = 0x0,
    GPIO_MODE_IN_FLOATING = 0x04,
    GPIO_MODE_IPD = 0x28,
    GPIO_MODE_IPU = 0x48,
    GPIO_MODE_OUT_OD = 0x14,
    GPIO_MODE_OUT_PP = 0x10,
    GPIO_MODE_AF_OD = 0x1C,
    GPIO_MODE_AF_PP = 0x18
} gpio_mode_e;

/* GPIO Init structure definition */
typedef struct
{
    uint16_t gpio_pins;             /* Specifies the GPIO pins to be configured.
                                    This parameter can be any value of @ref GPIO_pins_define */
    gpio_speed_e gpio_speed;  /* Specifies the speed for the selected pins.
                                    This parameter can be a value of @ref gpio_speed_e */
    gpio_mode_e gpio_mode;    /* Specifies the operating mode for the selected pins.
                                    This parameter can be a value of @ref gpio_mode_e */
} gpio_init_s;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    BIT_RESET = 0,
    BIT_SET = 1
} bit_action_e;

#define GPIO_NUM                    (16)

/* GPIO_pins_define */
#define GPIO_PIN_0                  ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_PIN_1                  ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_PIN_2                  ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_PIN_3                  ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_PIN_4                  ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_PIN_5                  ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_PIN_6                  ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_PIN_7                  ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_PIN_8                  ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_PIN_9                  ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_PIN_10                 ((uint16_t)0x0400)  /* Pin 10 selected */
#define GPIO_PIN_11                 ((uint16_t)0x0800)  /* Pin 11 selected */
#define GPIO_PIN_12                 ((uint16_t)0x1000)  /* Pin 12 selected */
#define GPIO_PIN_13                 ((uint16_t)0x2000)  /* Pin 13 selected */
#define GPIO_PIN_14                 ((uint16_t)0x4000)  /* Pin 14 selected */
#define GPIO_PIN_15                 ((uint16_t)0x8000)  /* Pin 15 selected */
#define GPIO_PIN_ALL                ((uint16_t)0xFFFF)  /* All pins selected */

/* GPIO_Remap_define */
/* PCFR1 */
#define GPIO_REMAP_SPI1             ((uint32_t)0x00000001)  /* SPI1 Alternate Function mapping */
#define GPIO_REMAP_I2C1             ((uint32_t)0x00000002)  /* I2C1 Alternate Function mapping */
#define GPIO_REMAP_USART1           ((uint32_t)0x00000004)  /* USART1 Alternate Function mapping low bit */
#define GPIO_REMAP_USART2           ((uint32_t)0x00000008)  /* USART2 Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART3    ((uint32_t)0x00140010)  /* USART3 Partial Alternate Function mapping */
#define GPIO_PARTIALREMAP1_USART3   ((uint32_t)0x00140020)  /* USART3 Partial1 Alternate Function mapping */
#define GPIO_FULLREMAP_USART3       ((uint32_t)0x00140030)  /* USART3 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_TIM1      ((uint32_t)0x00160040)  /* TIM1 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_TIM1         ((uint32_t)0x001600C0)  /* TIM1 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP1_TIM2     ((uint32_t)0x00180100)  /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PARTIALREMAP2_TIM2     ((uint32_t)0x00180200)  /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_FULLREMAP_TIM2         ((uint32_t)0x00180300)  /* TIM2 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_TIM3      ((uint32_t)0x001A0800)  /* TIM3 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_TIM3         ((uint32_t)0x001A0C00)  /* TIM3 Full Alternate Function mapping */
#define GPIO_REMAP_TIM4             ((uint32_t)0x00001000)  /* TIM4 Alternate Function mapping */
#define GPIO_REMAP1_CAN1            ((uint32_t)0x001D4000)  /* CAN1 Alternate Function mapping */
#define GPIO_REMAP2_CAN1            ((uint32_t)0x001D6000)  /* CAN1 Alternate Function mapping */
#define GPIO_REMAP_PD01             ((uint32_t)0x00008000)  /* PD01 Alternate Function mapping */
#define GPIO_REMAP_TIM5CH4_LSI      ((uint32_t)0x00200001)  /* LSI connected to TIM5 Channel4 input capture for calibration */
#define GPIO_REMAP_ADC1_ETRGINJ     ((uint32_t)0x00200002)  /* ADC1 External Trigger Injected Conversion remapping */
#define GPIO_REMAP_ADC1_ETRGREG     ((uint32_t)0x00200004)  /* ADC1 External Trigger Regular Conversion remapping */
#define GPIO_REMAP_ADC2_ETRGINJ     ((uint32_t)0x00200008)  /* ADC2 External Trigger Injected Conversion remapping */
#define GPIO_REMAP_ADC2_ETRGREG     ((uint32_t)0x00200010)  /* ADC2 External Trigger Regular Conversion remapping */
#define GPIO_REMAP_ETH              ((uint32_t)0x00200020)  /* Ethernet remapping (only for Connectivity line devices) */
#define GPIO_REMAP_CAN2             ((uint32_t)0x00200040)  /* CAN2 remapping (only for Connectivity line devices) */
#define GPIO_REMAP_MII_RMII_SEL     ((uint32_t)0x00200080)  /* MII or RMII selection */
#define GPIO_REMAP_SWJ_DISABLE      ((uint32_t)0x00300400)  /* Full SWJ Disabled (JTAG-DP + SW-DP) */
#define GPIO_REMAP_SPI3             ((uint32_t)0x00201000)  /* SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices) */
#define GPIO_REMAP_TIM2ITR1_PTP_SOF ((uint32_t)0x00202000)  /* Ethernet PTP output or USB OTG SOF (Start of Frame) connected
                                                               to TIM2 Internal Trigger 1 for calibration
                                                               (only for Connectivity line devices) */
#define GPIO_REMAP_PTP_PPS          ((uint32_t)0x00204000)  /* Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices) */

/* PCFR2 */
#define GPIO_REMAP_TIM8             ((uint32_t)0x80000004)  /* TIM8 Alternate Function mapping */
#define GPIO_PARTIALREMAP_TIM9      ((uint32_t)0x80130008)  /* TIM9 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_TIM9         ((uint32_t)0x80130010)  /* TIM9 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_TIM10     ((uint32_t)0x80150020)  /* TIM10 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_TIM10        ((uint32_t)0x80150040)  /* TIM10 Full Alternate Function mapping */
#define GPIO_REMAP_FSMC_NADV        ((uint32_t)0x80000400)  /* FSMC_NADV Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART4    ((uint32_t)0x80300001)  /* USART4 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_USART4       ((uint32_t)0x80300002)  /* USART4 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART5    ((uint32_t)0x80320004)  /* USART5 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_USART5       ((uint32_t)0x80320008)  /* USART5 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART6    ((uint32_t)0x80340010)  /* USART6 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_USART6       ((uint32_t)0x80340020)  /* USART6 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART7    ((uint32_t)0x80360040)  /* USART7 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_USART7       ((uint32_t)0x80360080)  /* USART7 Full Alternate Function mapping */
#define GPIO_PARTIALREMAP_USART8    ((uint32_t)0x80380100)  /* USART8 Partial Alternate Function mapping */
#define GPIO_FULLREMAP_USART8       ((uint32_t)0x80380200)  /* USART8 Full Alternate Function mapping */
#define GPIO_REMAP_USART1_HIGHBIT   ((uint32_t)0x80200400)  /* USART1 Alternate Function mapping high bit */


/* GPIO_Port_Sources */
#define GPIO_PORT_SOURCE_GPIOA        ((uint8_t)0x00)
#define GPIO_PORT_SOURCE_GPIOB        ((uint8_t)0x01)
#define GPIO_PORT_SOURCE_GPIOC        ((uint8_t)0x02)
#define GPIO_PORT_SOURCE_GPIOD        ((uint8_t)0x03)
#define GPIO_PORT_SOURCE_GPIOE        ((uint8_t)0x04)
#define GPIO_PORT_SOURCE_GPIOF        ((uint8_t)0x05)
#define GPIO_PORT_SOURCE_GPIOG        ((uint8_t)0x06)

/* GPIO_Pin_sources */
#define GPIO_PIN_SOURCE0             ((uint8_t)0x00)
#define GPIO_PIN_SOURCE1             ((uint8_t)0x01)
#define GPIO_PIN_SOURCE2             ((uint8_t)0x02)
#define GPIO_PIN_SOURCE3             ((uint8_t)0x03)
#define GPIO_PIN_SOURCE4             ((uint8_t)0x04)
#define GPIO_PIN_SOURCE5             ((uint8_t)0x05)
#define GPIO_PIN_SOURCE6             ((uint8_t)0x06)
#define GPIO_PIN_SOURCE7             ((uint8_t)0x07)
#define GPIO_PIN_SOURCE8             ((uint8_t)0x08)
#define GPIO_PIN_SOURCE9             ((uint8_t)0x09)
#define GPIO_PIN_SOURCE10            ((uint8_t)0x0A)
#define GPIO_PIN_SOURCE11            ((uint8_t)0x0B)
#define GPIO_PIN_SOURCE12            ((uint8_t)0x0C)
#define GPIO_PIN_SOURCE13            ((uint8_t)0x0D)
#define GPIO_PIN_SOURCE14            ((uint8_t)0x0E)
#define GPIO_PIN_SOURCE15            ((uint8_t)0x0F)

/* Ethernet_Media_Interface */
#define GPIO_ETH_MEDIA_INTERFACE_MII    ((u32)0x00000000)
#define GPIO_ETH_MEDIA_INTERFACE_RMII   ((u32)0x00000001)


void gpio_deinit(gpio_s* gpiox);
void gpio_afio_deinit(void);
void gpio_init(gpio_s* gpiox, gpio_init_s* gpio_init_struct);
void gpio_struct_init_default(gpio_init_s* gpio_init_struct);
uint8_t gpio_read_input_data_bit(gpio_s* gpiox, uint16_t gpio_pin);
uint16_t gpio_read_input_data(gpio_s* gpiox);
uint8_t gpio_read_output_data_bit(gpio_s* gpiox, uint16_t gpio_pin);
uint16_t gpio_read_output_data(gpio_s* gpiox);
void gpio_set_bits(gpio_s* gpiox, uint16_t gpio_pin);
void gpio_reset_bits(gpio_s* gpiox, uint16_t gpio_pin);
void gpio_write_bit(gpio_s* gpiox, uint16_t gpio_pin, bit_action_e BitVal);
void gpio_write(gpio_s* gpiox, uint16_t port_val);
void gpio_pin_lock_config(gpio_s* gpiox, uint16_t gpio_pin);
void gpio_event_output_config(uint8_t gpio_port_source, uint8_t gpio_pin_source);
void gpio_event_output_cmd(functional_state_e new_state);
void gpio_pin_remap_config(uint32_t gpio_remap, functional_state_e new_state);
void gpio_extiline_config(uint8_t gpio_port_source, uint8_t gpio_pin_source);
void gpio_eth_media_interface_config(uint32_t gpio_eth_media_interface);

#ifdef __cplusplus
}
#endif

#endif 







