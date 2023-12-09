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
uint8_t dc_set_direction[] = {STATE_SETTED, MOTOR_STOP};
int32_t dc0_set_speed[] = {STATE_SETTED, MOTOR_MAX_VALUE};
int32_t dc_set_speed[] = {STATE_SETTED, MOTOR_MAX_VALUE};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

// 0 and 1 LEFT SIDE
static inline void _dc0_dir_forward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC0_TIM1_CH1_OUTPUT_ENABLE);
}

static inline void _dc1_dir_forward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH3_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC1_TIM1_CH4_OUTPUT_ENABLE);
}

// 2 and 3 RIGHT  SIDE
static inline void _dc2_dir_forward_run(void)
{
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH1_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC2_TIM8_CH2_OUTPUT_ENABLE);
}

static inline void _dc3_dir_forward_run(void)
{
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH4_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC3_TIM8_CH3_OUTPUT_ENABLE);
}

static inline void _dc_forward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH3_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC1_TIM1_CH4_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH1_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC2_TIM8_CH2_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH4_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC3_TIM8_CH3_OUTPUT_ENABLE);
}

static inline void _dc0_dir_backward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC0_TIM1_CH2_OUTPUT_ENABLE);
}

static inline void _dc1_dir_backward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH4_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC1_TIM1_CH3_OUTPUT_ENABLE);
}

static inline void _dc2_dir_backward_run(void)
{
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH2_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC2_TIM8_CH1_OUTPUT_ENABLE);
}

static inline void _dc3_dir_backward_run(void)
{
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH3_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC3_TIM8_CH4_OUTPUT_ENABLE);
}

static inline void _dc_backward_run(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH4_OUTPUT_ENABLE);
	TIM1->CCER |= (uint16_t)(DC1_TIM1_CH3_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH2_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC2_TIM8_CH1_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH3_OUTPUT_ENABLE);
	TIM8->CCER |= (uint16_t)(DC3_TIM8_CH4_OUTPUT_ENABLE);
}

static inline void _dc0_stop(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
}

static inline void _dc_stop(void)
{
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH3_OUTPUT_ENABLE);
	TIM1->CCER &= (uint16_t)(~DC1_TIM1_CH4_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH1_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC2_TIM8_CH2_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH3_OUTPUT_ENABLE);
	TIM8->CCER &= (uint16_t)(~DC3_TIM8_CH4_OUTPUT_ENABLE);
}


//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_tim_init(void)
{
	gpio_init_t gpio_tim1_init_struct = {0};

	gpio_tim1_init_struct.GPIO_Pins = GPIO_Pin_8 | GPIO_Pin_9
									 | GPIO_Pin_10 | GPIO_Pin_11;
	gpio_tim1_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_tim1_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	gpio_init_t gpio_tim8_init_struct = {0};

	gpio_tim8_init_struct.GPIO_Pins = GPIO_Pin_6 | GPIO_Pin_7
									 | GPIO_Pin_8 | GPIO_Pin_9;
	gpio_tim8_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_tim8_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_Init(DC0_TIM1_GPIO, &gpio_tim1_init_struct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_Init(DC2_TIM8_GPIO, &gpio_tim8_init_struct);
}

static void _pwm_for_dc_init(uint8_t direction, uint32_t right_side_speed_percent,
		                     uint32_t left_side_speed_percent)
{
	uint32_t period = MOTOR_MAX_VALUE;
	uint32_t prescaler = MOTOR_PWM_PRESCALER - 1;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	tim_time_base_init_t tim_time_base_init_struct = {0};

	tim_time_base_init_struct.TIM_Period = period;
	tim_time_base_init_struct.TIM_Prescaler = prescaler;
	tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	tim_oc_init_t tim_dc_oc_init_struct = {0};

	tim_dc_oc_init_struct.TIM_OCMode = PWM_MODE;
	tim_dc_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	tim_dc_oc_init_struct.TIM_Pulse = right_side_speed_percent;
	tim_dc_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;

	if (FORWARD_DIRECTION == direction || BACKWARD_DIRECTION == direction)
	{
		TIM_TimeBaseInit(TIM1, &tim_time_base_init_struct);
		TIM_TimeBaseInit(TIM8, &tim_time_base_init_struct);
		TIM_OC1Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC2Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC3Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC4Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC1Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC2Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC3Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC4Init(TIM8, &tim_dc_oc_init_struct);
	}
	else
	{
		TIM_TimeBaseInit(TIM1, &tim_time_base_init_struct);
		TIM_OC1Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC2Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC3Init(TIM1, &tim_dc_oc_init_struct);
		TIM_OC4Init(TIM1, &tim_dc_oc_init_struct);

		tim_dc_oc_init_struct.TIM_Pulse = left_side_speed_percent;
		tim_time_base_init_struct.TIM_Prescaler = 2;
		TIM_TimeBaseInit(TIM8, &tim_time_base_init_struct);
		TIM_OC1Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC2Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC3Init(TIM8, &tim_dc_oc_init_struct);
		TIM_OC4Init(TIM8, &tim_dc_oc_init_struct);
	}

	_dc_stop();

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
}


/// \brief  Timer init for PWM mode
/// 	    duty cycle = pulse/period
/// \param	period - autoreload value
/// 		prescaler - f_pwm = f_apb2 / (psc + 1)
/// 		pulse - compare this value with period. If equal, change output.
/// \retval None
/// \return None
static void _tim_init(uint32_t period, uint32_t prescaler, uint32_t pulse)
{
	tim_oc_init_t tim_oc_init_struct = {0};
	tim_time_base_init_t tim_time_base_init_struct = {0};

	tim_time_base_init_struct.TIM_Period = period;
	tim_time_base_init_struct.TIM_Prescaler = prescaler;
	tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	tim_oc_init_struct.TIM_OCMode = PWM_MODE;
	tim_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	tim_oc_init_struct.TIM_Pulse = pulse;
	tim_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseInit(TIM1, &tim_time_base_init_struct);
	TIM_OC1Init(TIM1, &tim_oc_init_struct);
	TIM_OC2Init(TIM1, &tim_oc_init_struct);
	TIM_OC3Init(TIM1, &tim_oc_init_struct);
	TIM_OC4Init(TIM1, &tim_oc_init_struct);

	TIM_TimeBaseInit(TIM8, &tim_time_base_init_struct);
	TIM_OC1Init(TIM8, &tim_oc_init_struct);
	TIM_OC2Init(TIM8, &tim_oc_init_struct);
	TIM_OC3Init(TIM8, &tim_oc_init_struct);
	TIM_OC4Init(TIM8, &tim_oc_init_struct);

	_dc_stop();

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void dc_motor_driver_task(void *pvParameters)
{
	while(1)
	{
		if (SET_STATE == dc_set_direction[MOTOR_STATE_IDX_SET])
		{
			if (FORWARD_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc_forward_run();
				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (BACKWARD_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc_backward_run();
				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (MOTOR_STOP == dc_set_direction[MOTOR_STATE_IDX_DIR])
			{
				_dc_stop();
				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (LEFT_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
			{
				uint16_t left_speed = 900;
				uint16_t right_speed = 900;

				TIM_CtrlPWMOutputs(TIM1, DISABLE);
				TIM_CtrlPWMOutputs(TIM8, DISABLE);
				_pwm_for_dc_init(LEFT_DIRECTION, right_speed, left_speed);
				_dc_forward_run();
				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
			else if (RIGHT_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
			{
				uint16_t left_speed = 900;
				uint16_t right_speed = 900;

				TIM_CtrlPWMOutputs(TIM1, DISABLE);
				TIM_CtrlPWMOutputs(TIM8, DISABLE);
				_pwm_for_dc_init(RIGHT_DIRECTION, right_speed, left_speed);
				_dc_forward_run();
				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
			}
		}
		if (SET_STATE == dc_set_speed[MOTOR_STATE_IDX_SET])
		{
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			TIM_CtrlPWMOutputs(TIM8, DISABLE);
			_pwm_for_dc_init(FORWARD_DIRECTION,
					dc_set_speed[MOTOR_STATE_IDX_SPEED],
					dc_set_speed[MOTOR_STATE_IDX_SPEED]);
			dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;

			dc_set_speed[MOTOR_STATE_IDX_SET] = STATE_SETTED;
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
	_tim_init(MOTOR_MAX_VALUE, MOTOR_PWM_PRESCALER-1, MOTOR_MAX_VALUE);

    xTaskCreate((TaskFunction_t )dc_motor_driver_task,
				(const char*    )"cmd interface listening",
				(uint16_t       )DC_MOTOR_DRIVER_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )DC_MOTOR_DRIVER_TASK_PRIORITY,
				(TaskHandle_t*  )&dc_motor_driver_task_handler);
}
