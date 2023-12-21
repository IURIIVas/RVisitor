/// \file hw_201.h
/// \brief HW-201 IR sensor library
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef HW_201_H_
#define HW_201_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------


//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------


//------------------------------------------------ Function prototypes -------------------------------------------------

void hw_201_rcc_gpio_clk_init(uint32_t RCC_GPIO);
void hw_201_gpio_signal_in_init(gpio_s *GPIO_PORT, uint16_t GPIO_IN_PIN);
uint8_t hw_201_gpio_signal_in_get(gpio_s *GPIO_PORT, uint16_t GPIO_IN_PIN);


#endif /* HW_201_H_ */
