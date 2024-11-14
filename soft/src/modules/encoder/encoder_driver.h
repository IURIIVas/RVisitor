/// \file encoder_driver.h
/// \brief encoder driver header files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef ENCODER_DRIVER_H_
#define ENCODER_DRIVER_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define ENC_NUM                               (4)
#define ENC_NUM_PER_DC                        (2)

#define ENC_TIM2_TIM5_GPIO                    (GPIOA)
#define ENC_TIM4_GPIO                         (GPIOB)
#define ENC_TIM9_GPIO                         (GPIOD)

#define RCC_TIM2_TIM5_GPIO                    (RCC_APB2Periph_GPIOA)
#define RCC_TIM2_TIM4_GPIO                    (RCC_APB2Periph_GPIOB)
#define RCC_TIM9_GPIO                         (RCC_APB2Periph_GPIOD)

#define ENC_COUNT_VALUE                       (0xffff)
#define ENC_TICS_ONE_PERIOD                   (620)
#define ENC_PPR                               (7)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef enum
{
    FL_ENC = 0, // TIM 9
    RL_ENC = 1,
    FR_ENC = 2,
    RR_ENC = 3
} encoders_e;


//------------------------------------------------ Function prototypes -------------------------------------------------

void gpio_tim_enc_init(void);
void enc_init(void);
double get_speed_rpm(encoders_e);

#endif
