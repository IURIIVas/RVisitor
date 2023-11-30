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

void hc_sr04_rcc_gpio_clk_init(uint32_t RCC_GPIO);
void hc_sr04_trig_base_tim_clk_init(uint32_t RCC_TIM);
void hc_sr04_trig_gpio_init(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_TRIG_PIN);
void hc_sr04_trig_base_tim_init(TIM_TypeDef *TIMER);

#endif /* HC_SR04_H_ */