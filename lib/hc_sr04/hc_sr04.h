/// \file hc_sr04.h
/// \brief HC-SR04 sensor library
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef HC_SR04_H_
#define HC_SR04_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

void hc_sr04_trig_gpio_init(gpio_s *GPIO_PORT, uint16_t GPIO_TRIG_PIN);
void hc_sr04_trig_base_tim_init(TIM_TypeDef *TIMER);
void hc_sr04_echo_gpio_init(gpio_s *GPIO_PORT, uint16_t GPIO_ECHO_PIN);
void hc_sr04_echo_tim_init(TIM_TypeDef *TIMER);

#endif /* HC_SR04_H_ */
