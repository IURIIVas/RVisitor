/// \file dc_motor_driver.c
/// \brief DC_Motor driver source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "dc_motor_driver.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t dc_motor_driver_task_handler;
uint8_t dc0_set_direction[] = {STATE_SETTED, MOTOR_STOP};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

static inline void _dc0_dir_forward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC0_TIM1_CH1_OUTPUT_ENABLE);
}

static inline void _dc0_dir_backward_run(void)
{
	TIM1->CCER &= (uint16_t)(DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
}

static inline void _dc0_stop(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
}


//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_tim_init(void)
{
	gpio_init_t gpio_dc0_tim1_init_struct = {0};

	gpio_dc0_tim1_init_struct.GPIO_Pins = GPIO_Pin_8 | GPIO_Pin_9
									 | GPIO_Pin_10 | GPIO_Pin_11;
	gpio_dc0_tim1_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_dc0_tim1_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_Init(DC0_TIM1_GPIO, &gpio_dc0_tim1_init_struct);
}

/// \brief Timer init for PWM mode
/// \param	period - perido value
/// 		prescaler - prescaler value
/// 		pulse - pulse value
/// \retval None
/// \return None
static void _tim_init(uint16_t period, uint16_t prescaler, uint16_t pulse)
{
	tim_oc_init_t tim_1_oc_init_struct = {0};
	tim_time_base_init_t tim_1_time_base_init_struct = {0};

	tim_1_time_base_init_struct.TIM_Period = period;
	tim_1_time_base_init_struct.TIM_Prescaler = prescaler;
	tim_1_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_1_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	tim_1_oc_init_struct.TIM_OCMode = PWM_MODE;
	tim_1_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	tim_1_oc_init_struct.TIM_Pulse = pulse;
	tim_1_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInit(TIM1, &tim_1_time_base_init_struct);
	TIM_OC1Init(TIM1, &tim_1_oc_init_struct);
	TIM_OC2Init(TIM1, &tim_1_oc_init_struct);

	_dc0_stop();

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void dc_motor_driver_task(void *pvParameters)
{
	while(1)
	{
		if (SET_STATE == dc0_set_direction[MOTOR_STATE_IDX_SET])
		{
			if (FORWARD_DIRECTION == dc0_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc0_dir_forward_run();
				dc0_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (BACKWARD_DIRECTION == dc0_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc0_dir_backward_run();
				dc0_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (MOTOR_STOP == dc0_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc0_stop();
				dc0_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
		}
	}
}

/// \brief
/// \param
/// \retval
/// \return
void dc_motor_driver_task_init(void)
{
	_gpio_tim_init();
	_tim_init(100, 48000-1, 50);

    xTaskCreate((TaskFunction_t )dc_motor_driver_task,
				(const char*    )"cmd interface listening",
				(uint16_t       )DC_MOTOR_DRIVER_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )DC_MOTOR_DRIVER_TASK_PRIORITY,
				(TaskHandle_t*  )&dc_motor_driver_task_handler);
}
