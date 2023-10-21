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

#define CMD_IFACE_UART					  (USART2)
#define CMD_IFACE_BAUDRATE				  (115200)
#define CMD_IFACE_GPIO					  (GPIOA)
#define CMD_IFACE_GPIO_TX_PIN			  (GPIO_Pin_2)
#define CMD_IFACE_GPIO_RX_PIN			  (GPIO_Pin_3)
#define CMD_IFACE_RCC_APB1_UART  		  (RCC_APB1Periph_USART2)
#define CMD_IFACE_RCC_APB2_GPIO  		  (RCC_APB2Periph_GPIOA)
#define CMD_IFACE_NVIC_IRQ_CHANNEL		  (USART2_IRQn)

#define CMD_IFACE_LISTENING_PRIO     	  (5)
#define CMD_IFACE_LISTENING_STK_SIZE      (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void cmd_iface_listening_task_init();



#endif /* CMD_INTERFACE_H_ */
