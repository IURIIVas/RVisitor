/// \file cmd_interface.c
/// \brief UART command interface
/// \author 1jura.vas@gmail.com
///
/// \details
///

#ifndef CMD_INTERFACE_H_
#define CMD_INTERFACE_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "uart.h"
#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define CMD_MAX_LEN						  (50)

#define CMD_IFACE_UART					  (UART5)
#define CMD_IFACE_BAUDRATE				  (115200)
#define CMD_IFACE_GPIO					  (GPIOB)
#define CMD_IFACE_GPIO_TX_PIN			  (GPIO_Pin_4)
#define CMD_IFACE_GPIO_RX_PIN			  (GPIO_Pin_5)
#define CMD_IFACE_RCC_APB1_UART  		  (RCC_APB1Periph_UART5)
#define CMD_IFACE_RCC_APB2_GPIO  		  (RCC_APB2Periph_GPIOB)
#define CMD_IFACE_NVIC_IRQ_CHANNEL		  (UART5_IRQn)

#define CMD_IFACE_LISTENING_PRIO     	  (5)
#define CMD_IFACE_LISTENING_STK_SIZE      (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct params
{
    uint8_t cliff_enable;
    uint8_t obstacles_enable;
    uint8_t overcurrent_enable;
    uint8_t speed_get_inf_enable;
    uint8_t sens_survey_inf_enable;
} module_params_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void cmd_iface_listening_task_init();



#endif /* CMD_INTERFACE_H_ */
