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
double speed_ms[ENC_NUM] = {0, 0, 0, 0};
int32_t dc0_set_speed = 0;
int32_t dc1_set_speed = 0;
int32_t dc2_set_speed = 0;
int32_t dc3_set_speed = 0;
uint8_t dc_set_speed = STATE_SETTED;
double target_speed_lin_ms = 0;
double target_speed_ang_rads = 0;

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

static void _gpio_tim_dc_init(void)
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

static void _gpio_tim_enc_init(void)
{
    AFIO->PCFR1 |= GPIO_FullRemap_TIM2;
    gpio_init_t gpio_tim2_tim3_tim5_init_struct = {0};

    gpio_tim2_tim3_tim5_init_struct.GPIO_Pins = GPIO_Pin_0 | GPIO_Pin_1
                                     | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_13;
    gpio_tim2_tim3_tim5_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim2_tim3_tim5_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_t gpio_tim4_init_struct = {0};

    gpio_tim4_init_struct.GPIO_Pins = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_3;
    gpio_tim4_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim4_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init(ENC_TIM2_TIM3_TIM5_GPIO, &gpio_tim2_tim3_tim5_init_struct);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(ENC_TIM4_GPIO, &gpio_tim4_init_struct);
}

/// \brief TIM3 - DC0 (PA6, PA7) (-) (ONLY -1)
///        TIM5 - DC3 (PA2, PA3) (NOT WORKING)
///        TIM2 - DC2 (PA0, PA1) (WORKS WITH TIM5) (+/-)
///        TIM4 - DC1 (PB6, PB7) (+)
static void _enc_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    tim_time_base_init_t tim_time_base_init_struct = {0};

    tim_time_base_init_struct.TIM_Period = ENC_COUNT_VALUE;
    tim_time_base_init_struct.TIM_Prescaler = 0;
    tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM3, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM4, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM5, &tim_time_base_init_struct);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM5, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    TIM_SetCounter(TIM2, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM3, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM4, ENC_COUNT_VALUE / 2);
    TIM_SetCounter(TIM5, ENC_COUNT_VALUE / 2);
}

/// \brief  Timer init for PWM mode
///         duty cycle = pulse/period
///         period - autoreload value
///         prescaler - f_pwm = f_apb2 / (psc + 1)
///         pulse - compare this value with period. If equal, change output.
/// \param
///         dc_0_speed - speed for dc0
///         dc_1_speed - speed for dc1
///         dc_2_speed - speed for dc2
///         dc_3_speed - speed for dc3
/// \retval None
/// \return None
static void _pwm_for_dc_init(uint32_t dc_0_speed, uint32_t dc_1_speed,
                             uint32_t dc_2_speed, uint32_t dc_3_speed)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	uint32_t period = MOTOR_MAX_VALUE;
	uint32_t prescaler = MOTOR_PWM_PRESCALER - 1;

	tim_time_base_init_t tim_time_base_init_struct = {0};

	tim_time_base_init_struct.TIM_Period = period;
	tim_time_base_init_struct.TIM_Prescaler = prescaler;
	tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

	tim_oc_init_t tim_dc_oc_init_struct = {0};

	tim_dc_oc_init_struct.TIM_OCMode = PWM_MODE;
	tim_dc_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
	tim_dc_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_TimeBaseInit(TIM1, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM8, &tim_time_base_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_0_speed;
    TIM_OC1Init(TIM1, &tim_dc_oc_init_struct);
    TIM_OC2Init(TIM1, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_1_speed;
    TIM_OC3Init(TIM1, &tim_dc_oc_init_struct);
    TIM_OC4Init(TIM1, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_2_speed;
    TIM_OC1Init(TIM8, &tim_dc_oc_init_struct);
    TIM_OC2Init(TIM8, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_3_speed;
    TIM_OC3Init(TIM8, &tim_dc_oc_init_struct);
    TIM_OC4Init(TIM8, &tim_dc_oc_init_struct);

	_dc_stop();

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Disable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
}

static void _pwm_for_dc_change_speed(uint32_t dc_0_speed, uint32_t dc_1_speed,
                                      uint32_t dc_2_speed, uint32_t dc_3_speed)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM8, DISABLE);

    TIM1->CH1CVR = dc_0_speed;
    TIM1->CH2CVR = dc_0_speed;

    TIM1->CH3CVR = dc_1_speed;
    TIM1->CH4CVR = dc_1_speed;

    TIM8->CH1CVR = dc_2_speed;
    TIM8->CH2CVR = dc_2_speed;

    TIM8->CH3CVR = dc_3_speed;
    TIM8->CH4CVR = dc_3_speed;

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}

/// \brief TIM3 - DC0 (PA6, PA7) (-) (ONLY -1)
///        TIM5 - DC3 (PA2, PA3) (NOT WORKING)
///        TIM2 - DC2 (PA0, PA1) (WORKS WITH TIM5) (+/-)
///        TIM4 - DC1 (PB6, PB7) (+)
static void _get_speed_ms(void)
{
    for (size_t i = 0; i < ENC_NUM; i++)
    {
        int32_t tiks = 0;
        switch (i)
        {
        case 0:
            tiks = TIM3->CNT - ENC_COUNT_VALUE / 2;
            TIM3->CNT = ENC_COUNT_VALUE / 2;
            break;
        case 1:
            tiks = TIM4->CNT - ENC_COUNT_VALUE / 2;
            TIM4->CNT = ENC_COUNT_VALUE / 2;
            break;
        case 2:
            tiks = TIM2->CNT - ENC_COUNT_VALUE / 2;
            TIM2->CNT = ENC_COUNT_VALUE / 2;
            break;
        case 3:
            tiks = TIM5->CNT - ENC_COUNT_VALUE / 2;
            TIM5->CNT = ENC_COUNT_VALUE / 2;
            break;
        }
        if (tiks < 0)
        {
            tiks = -tiks;
        }
        double wheels_pass = (double)tiks / ENC_TICS_ONE_WHEEL;
        double distance_m = wheels_pass * WHEEL_C_CM / 100;
        double time_s = 2;
        speed_ms[i] = distance_m / time_s;
    }
}

static void _pid_calculate(m_pid_s *pid)
{
    double input = 0;
    double error = 0;
    double d_input = 0;
    double output = 0;

    input = pid->input;
    error = pid->target - input;
    d_input = input - pid->last_input;

    pid->output_sum += (pid->ki * error);

    output += pid->output_sum - (pid->kd * d_input);

    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    pid->output = output;
    pid->last_input = input;
}

//---------------------------------------------------- Functions -------------------------------------------------------

void dc_motor_driver_task(void *pvParameters)
{
    m_pid_s pid_0 = {.kp = 1, .ki = 1, .kd = 1, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 50, .output = 0,
                   .output_sum = 0, .target = target_speed_lin_ms};
    m_pid_s pid_1 = {.kp = 1, .ki = 1, .kd = 1, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 50, .output = 0,
                   .output_sum = 0, .target = target_speed_lin_ms};
    m_pid_s pid_2 = {.kp = 1, .ki = 1, .kd = 1, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 50, .output = 0,
                   .output_sum = 0, .target = target_speed_lin_ms};
    m_pid_s pid_3 = {.kp = 1, .ki = 1, .kd = 1, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 50, .output = 0,
                   .output_sum = 0, .target = target_speed_lin_ms};

	while(1)
	{
	    if (target_speed_lin_ms < 0)
	    {
	        _dc_backward_run();
	    }
	    else if (target_speed_lin_ms > 0)
	    {
            _dc_forward_run();
        }
	    else
	    {
	        _dc_stop();
	        continue;
	    }

	    _get_speed_ms();

	    pid_0.input = speed_ms[0];
	    pid_0.target = target_speed_lin_ms;
	    _pid_calculate(&pid_0);
	    dc0_set_speed = pid_0.output;

        pid_1.input = speed_ms[1];
        pid_1.target = target_speed_lin_ms;
        _pid_calculate(&pid_1);
        dc1_set_speed = pid_1.output;

        pid_2.input = speed_ms[2];
        pid_2.target = target_speed_lin_ms;
        _pid_calculate(&pid_2);
        dc2_set_speed = pid_2.output;

        pid_3.input = speed_ms[3];
        pid_3.target = target_speed_lin_ms;
        _pid_calculate(&pid_3);
        dc3_set_speed = pid_3.output;

	    _pwm_for_dc_change_speed(dc0_set_speed, dc1_set_speed,
	                             dc2_set_speed, dc3_set_speed);

	    vTaskDelay(100 / portTICK_PERIOD_MS);
//		if (SET_STATE == dc_set_direction[MOTOR_STATE_IDX_SET])
//		{
//			if (FORWARD_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
//			{
//				_dc_forward_run();
//				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
//			}
//			else if (BACKWARD_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
//			{
//				_dc_backward_run();
//				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
//			}
//			else if (MOTOR_STOP == dc_set_direction[MOTOR_STATE_IDX_DIR])
//			{
//				_dc_stop();
//				dc_set_direction[MOTOR_STATE_IDX_SET] = STATE_SETTED;
//			}
//			else if (LEFT_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
//			{
//			}
//			else if (RIGHT_DIRECTION == dc_set_direction[MOTOR_STATE_IDX_DIR])
//			{
//			}
//		}
//		if (SET_STATE == dc_set_speed)
//		{
//		    _get_speed_ms();
//			_pwm_for_dc_change_speed(dc0_set_speed, dc1_set_speed,
//			                         dc2_set_speed, dc3_set_speed);
//			dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
//
//			dc_set_speed = STATE_SETTED;
//		}



	}
}

/// \brief
/// \param
/// \retval
/// \return
void dc_motor_driver_task_init(void)
{
	_gpio_tim_dc_init();
	_gpio_tim_enc_init();
	_pwm_for_dc_init(0, 0, 0, 0);
	_enc_init();

    xTaskCreate((TaskFunction_t )dc_motor_driver_task,
				(const char*    )"dc controller",
				(uint16_t       )DC_MOTOR_DRIVER_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )DC_MOTOR_DRIVER_TASK_PRIORITY,
				(TaskHandle_t*  )&dc_motor_driver_task_handler);
}
