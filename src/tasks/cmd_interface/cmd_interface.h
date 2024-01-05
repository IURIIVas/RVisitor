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

#define CMD_INF_SURVEY_PERIOD_MS          (200)

#define PARAM_CLIFF_ENABLE_BIT            (0)
#define PARAM_OBSTACLES_ENABLE_BIT        (1)
#define PARAM_OVERCURRENT_ENABLE_BIT      (2)
#define PARAM_SPEED_GET_INF_ENABLE_BIT    (3)
#define PARAM_SENS_SURVEY_INF_ENABLE_BIT  (4)
#define PARAM_FLAGS_ENABLE_BIT            (5)

#define PARAM_CLIFF_ENABLE                (1 << PARAM_CLIFF_ENABLE_BIT)
#define PARAM_OBSTACLES_ENABLE            (1 << PARAM_OBSTACLES_ENABLE_BIT)
#define PARAM_OVERCURRENT_ENABLE          (1 << PARAM_OVERCURRENT_ENABLE_BIT)
#define PARAM_SPEED_GET_INF_ENABLE        (1 << PARAM_SPEED_GET_INF_ENABLE_BIT)
#define PARAM_SENS_SURVEY_INF_ENABLE      (1 << PARAM_SENS_SURVEY_INF_ENABLE_BIT)
#define PARAM_FLAGS_ENABLE                (1 << PARAM_FLAGS_ENABLE_BIT)


//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct
{
    uint16_t params;
} module_params_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void cmd_iface_listening_task_init();



#endif /* CMD_INTERFACE_H_ */
