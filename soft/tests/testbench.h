/// \file test_flash.h
/// \brief test flash w25q32 header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef TESTBENCH_H_
#define TESTBENCH_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define CMD_IFACE_UART                    (UART5)
#define CMD_IFACE_BAUDRATE                (115200)
#define CMD_IFACE_GPIO                    (GPIOB)
#define CMD_IFACE_GPIO_TX_PIN             (GPIO_Pin_4)
#define CMD_IFACE_GPIO_RX_PIN             (GPIO_Pin_5)
#define CMD_IFACE_RCC_APB1_UART           (RCC_APB1Periph_UART5)
#define CMD_IFACE_RCC_APB2_GPIO           (RCC_APB2Periph_GPIOB)
#define CMD_IFACE_NVIC_IRQ_CHANNEL        (UART5_IRQn)

#define FLASH_TEST                        (1)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

extern uint32_t test_num;

//------------------------------------------------ Function prototypes -------------------------------------------------

void testbench_init(void);

#endif /* TESTBENCH_H_ */
