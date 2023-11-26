/// \file hw_201.c
/// \brief HW-201 IR sensor library
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "hw_201.h"
#include "gpio.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

/// \brief Choose sensor by external MUX
/// \param GPIO_PORT - Selected Port, GPIO_IN_PIN - PIN, when OUT signal from sensor connected
/// \retval uint8_t
/// \return Signal from GPIO_IN_PIN
inline uint8_t hw_201_gpio_signal_in_get(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_IN_PIN)
{
	return GPIO_ReadInputDataBit(GPIO_PORT, GPIO_IN_PIN);
}

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief CLK Enable to selected GPIO
/// \param APB Periph addr
/// \retval None
/// \return None
void hw_201_rcc_gpio_clk_init(uint32_t RCC_GPIO)
{
	RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
}

/// \brief Choose sensor by external MUX
/// \param GPIO_PORT - Selected Port, GPIO_IN_PIN - PIN, when OUT signal from sensor connected
/// \retval None
/// \return None
void hw_201_gpio_signal_in_init(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_IN_PIN)
{
	gpio_init_t gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = GPIO_IN_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIO_PORT, &gpio_init_struct);
}

