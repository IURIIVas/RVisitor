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
/// \return Signal from GPIO_IN_PIN
inline uint8_t hw_201_gpio_signal_in_get(gpio_s *GPIO_PORT, uint16_t GPIO_IN_PIN)
{
	return gpio_read_input_data_bit(GPIO_PORT, GPIO_IN_PIN);
}

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief CLK Enable to selected GPIO
/// \param APB Periph addr
/// \return None
void hw_201_rcc_gpio_clk_init(uint32_t RCC_GPIO)
{
	RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
}

/// \brief Choose sensor by external MUX
/// \param GPIO_PORT - Selected Port, GPIO_IN_PIN - PIN, when OUT signal from sensor connected
/// \return None
void hw_201_gpio_signal_in_init(gpio_s *GPIO_PORT, uint16_t GPIO_IN_PIN)
{
	gpio_init_s gpio_init_struct = {0};

	gpio_init_struct.gpio_pins = GPIO_IN_PIN;
	gpio_init_struct.gpio_mode = GPIO_MODE_IN_FLOATING;
	gpio_init(GPIO_PORT, &gpio_init_struct);
}

