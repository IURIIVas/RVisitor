/// \file encoder_driver.c
/// \brief tim drive as an encoder
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "encoder_driver.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

void gpio_tim_enc_init(void)
{
    AFIO->PCFR1 |= GPIO_FULLREMAP_TIM2;
    AFIO->PCFR2 |= GPIO_FULLREMAP_TIM9;
    gpio_init_s gpio_tim2_tim5_init_struct = {0};

    gpio_tim2_tim5_init_struct.gpio_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_15;
    gpio_tim2_tim5_init_struct.gpio_mode = GPIO_MODE_AF_PP;
    gpio_tim2_tim5_init_struct.gpio_speed = GPIO_SPEED_50MHZ;

    gpio_init_s gpio_tim2_tim4_init_struct = {0};

    gpio_tim2_tim4_init_struct.gpio_pins = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_3;
    gpio_tim2_tim4_init_struct.gpio_mode = GPIO_MODE_AF_PP;
    gpio_tim2_tim4_init_struct.gpio_speed = GPIO_SPEED_50MHZ;

    gpio_init_s gpio_tim9_init_struct = {0};

    gpio_tim9_init_struct.gpio_pins = GPIO_PIN_9 | GPIO_PIN_11;
    gpio_tim9_init_struct.gpio_mode = GPIO_MODE_AF_PP;
    gpio_tim9_init_struct.gpio_speed = GPIO_SPEED_50MHZ;

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM5_GPIO, ENABLE);
    gpio_init(ENC_TIM2_TIM5_GPIO, &gpio_tim2_tim5_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM4_GPIO, ENABLE);
    gpio_init(ENC_TIM4_GPIO, &gpio_tim2_tim4_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM9_GPIO, ENABLE);
    gpio_init(ENC_TIM9_GPIO, &gpio_tim9_init_struct);
}

/// \brief TIM3 - DC0 (PA6, PA7)
///        TIM5 - DC3 (PA2, PA3)
///        TIM2 - DC2 (PA0, PA1)
///        TIM4 - DC1 (PB6, PB7)
void enc_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    tim_time_base_init_s tim_time_base_init_struct = {0};

    tim_time_base_init_struct.TIM_Period = ENC_COUNT_VALUE;
    tim_time_base_init_struct.TIM_Prescaler = 0;
    tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM9, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM4, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM5, &tim_time_base_init_struct);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM9, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM5, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    TIM_SetCounter(TIM2, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM9, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM4, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM5, ENC_COUNT_VALUE / 2);
}


double get_speed_rpm(encoders_e enc)
{

    const double interval = 0.1;
	int32_t ticks = 0;

	switch (enc)
	{
    case FL_ENC:
        ticks = TIM9->CNT - ENC_COUNT_VALUE / 2;
        TIM9->CNT = ENC_COUNT_VALUE / 2;
        break;
	case RL_ENC:
		ticks = TIM4->CNT - ENC_COUNT_VALUE / 2;
		TIM4->CNT = ENC_COUNT_VALUE / 2;
		break;
	case FR_ENC:
		ticks = TIM5->CNT - ENC_COUNT_VALUE / 2;
		TIM5->CNT = ENC_COUNT_VALUE / 2;
		break;
	case RR_ENC:
		ticks = TIM2->CNT - ENC_COUNT_VALUE / 2;
		TIM2->CNT = ENC_COUNT_VALUE / 2;
		break;
	}

	if (ticks < 0)
	{
		ticks = -ticks;
	}

	double speed_rpm = 60 * ticks / (ENC_TICS_ONE_PERIOD * interval);

	return speed_rpm;
}

