/// \file hc_sr04.c
/// \brief HC-SR04 sensor library
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "hc_sr04.h"
#include "gpio.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief CLK Enable to selected GPIO
/// \param APB Periph addr
/// \retval None
/// \return None
void hc_sr04_rcc_gpio_clk_init(uint32_t RCC_GPIO)
{
	RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
}

/// \brief TRIG PIN init to selected GPIO
/// \param GPIO Port, GPIO Trig Pin
/// \retval None
/// \return None
void hc_sr04_trig_gpio_init(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_TRIG_PIN)
{
	gpio_init_t gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = GPIO_TRIG_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_PORT, &gpio_init_struct);

	GPIO_SetBits(GPIO_PORT, GPIO_TRIG_PIN);
}

/// \brief CLK Enable to selected timer
/// \param APB Periph addr
/// \retval None
/// \return None
void hc_sr04_trig_base_tim_clk_init(uint32_t RCC_TIM)
{
	RCC_APB1PeriphClockCmd(RCC_TIM, ENABLE);
}

/// \brief Init timer to count 10 microsecond for trig signal
/// \param Timer
/// \retval None
/// \return None
void hc_sr04_trig_base_tim_init(TIM_TypeDef *TIMER)
{
	tim_time_base_init_t time_base_init_struct = {0};
	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);

	uint32_t prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 1000000 - 1;

	time_base_init_struct.TIM_Prescaler = prescaler;
	time_base_init_struct.TIM_Period = 0xffff;
	time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;
	time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_ARRPreloadConfig(TIMER, ENABLE);

	TIM_TimeBaseInit(TIMER, &time_base_init_struct);
	TIM_Cmd(TIMER, ENABLE);
}

/// \brief ECHO PIN input init to selected GPIO
/// \param GPIO Port, GPIO Echo Pin
/// \retval None
/// \return None
void hc_sr04_echo_gpio_init(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_ECHO_PIN)
{
	gpio_init_t gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = GPIO_ECHO_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_PORT, &gpio_init_struct);
}

/// \brief Timer channel enable to echo input
/// \param Timer
/// \retval None
/// \return None
void hc_sr04_echo_tim_init(TIM_TypeDef *TIMER)
{
	tim_time_base_init_t time_base_init_struct = {0};
	tim_ic_init_t time_ic_init_struct = {0};
	nvic_init_t nvic_init_struct = {0};

	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);

	uint32_t prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 1000000 - 1;

	time_base_init_struct.TIM_Prescaler = prescaler;
	time_base_init_struct.TIM_Period = 0xffff;
	time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;
	time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMER, &time_base_init_struct);

	time_ic_init_struct.TIM_Channel = TIM_Channel_3;
	time_ic_init_struct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	time_ic_init_struct.TIM_ICFilter = 0x00;
	time_ic_init_struct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	time_ic_init_struct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIMER, &time_ic_init_struct);

	nvic_init_struct.NVIC_IRQChannel = TIM3_IRQn;
	nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init_struct.NVIC_IRQChannelSubPriority = 0;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init_struct);

	TIM_ITConfig(TIMER, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

	TIM_SelectInputTrigger(TIMER, TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(TIMER, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIMER, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIMER, ENABLE);
}
