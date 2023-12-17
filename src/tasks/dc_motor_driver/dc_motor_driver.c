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
#include "odometry.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t dc_motor_driver_task_handler;

dc_motor_set_s dc_motor_set = {.target_speed_rpm = {0, 0},

                               .dc_motor_speed_pwm = {0, 0, 0, 0},
                               .dc_motor_speed_rpm = {0, 0, 0, 0} };

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

static inline void _dc_ls_backward_run(void)
{
    LS_PWM_TIM->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER |= (uint16_t)(DC0_TIM1_CH2_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER &= (uint16_t)(~DC1_TIM1_CH4_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER |= (uint16_t)(DC1_TIM1_CH3_OUTPUT_ENABLE);
}

static inline void _dc_rs_backward_run(void)
{
    RS_PWM_TIM->CCER &= (uint16_t)(~DC2_TIM10_CH2_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER |= (uint16_t)(DC2_TIM10_CH1_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC3_TIM10_CH3_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER |= (uint16_t)(DC3_TIM10_CH4_OUTPUT_ENABLE);
}

static inline void _dc_ls_forward_run(void)
{
    LS_PWM_TIM->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER |= (uint16_t)(DC0_TIM1_CH1_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER &= (uint16_t)(~DC1_TIM1_CH3_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER |= (uint16_t)(DC1_TIM1_CH4_OUTPUT_ENABLE);
}

static inline void _dc_rs_forward_run(void)
{
    RS_PWM_TIM->CCER &= (uint16_t)(~DC2_TIM10_CH1_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER |= (uint16_t)(DC2_TIM10_CH2_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC3_TIM10_CH4_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER |= (uint16_t)(DC3_TIM10_CH3_OUTPUT_ENABLE);
}

static inline void _dc_stop(void)
{
    LS_PWM_TIM->CCER &= (uint16_t)(~DC0_TIM1_CH1_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER &= (uint16_t)(~DC0_TIM1_CH2_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER &= (uint16_t)(~DC1_TIM1_CH3_OUTPUT_ENABLE);
    LS_PWM_TIM->CCER &= (uint16_t)(~DC1_TIM1_CH4_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC2_TIM10_CH1_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC2_TIM10_CH2_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC3_TIM10_CH3_OUTPUT_ENABLE);
    RS_PWM_TIM->CCER &= (uint16_t)(~DC3_TIM10_CH4_OUTPUT_ENABLE);
}

static inline void _dc_forward_run(void)
{
    _dc_ls_forward_run();
    _dc_rs_forward_run();
}

static inline void _dc_backward_run(void)
{
    _dc_ls_backward_run();
    _dc_rs_backward_run();
}


//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_tim_dc_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    AFIO->PCFR2 |= GPIO_FullRemap_TIM10;

	gpio_init_t gpio_left_side_init_struct = {0};

	gpio_left_side_init_struct.GPIO_Pins = GPIO_Pin_8 | GPIO_Pin_9
									 | GPIO_Pin_10 | GPIO_Pin_11;
	gpio_left_side_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_left_side_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	gpio_init_t gpio_right_side_init_struct = {0};

	gpio_right_side_init_struct.GPIO_Pins = GPIO_Pin_1 | GPIO_Pin_3
									 | GPIO_Pin_5 | GPIO_Pin_7;
	gpio_right_side_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_right_side_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_Init(LS_GPIO, &gpio_left_side_init_struct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_Init(RS_GPIO, &gpio_right_side_init_struct);
}

static void _gpio_tim_enc_init(void)
{
    AFIO->PCFR1 |= GPIO_FullRemap_TIM2;
    AFIO->PCFR2 |= GPIO_FullRemap_TIM9;
    gpio_init_t gpio_tim2_tim5_init_struct = {0};

    gpio_tim2_tim5_init_struct.GPIO_Pins = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_15;
    gpio_tim2_tim5_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim2_tim5_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_t gpio_tim2_tim4_init_struct = {0};

    gpio_tim2_tim4_init_struct.GPIO_Pins = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_3;
    gpio_tim2_tim4_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim2_tim4_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_t gpio_tim9_init_struct = {0};

    gpio_tim9_init_struct.GPIO_Pins = GPIO_Pin_9 | GPIO_Pin_11;
    gpio_tim9_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim9_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM5_GPIO, ENABLE);
    GPIO_Init(ENC_TIM2_TIM5_GPIO, &gpio_tim2_tim5_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM4_GPIO, ENABLE);
    GPIO_Init(ENC_TIM4_GPIO, &gpio_tim2_tim4_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM9_GPIO, ENABLE);
    GPIO_Init(ENC_TIM4_GPIO, &gpio_tim9_init_struct);
}

/// \brief TIM3 - DC0 (PA6, PA7)
///        TIM5 - DC3 (PA2, PA3)
///        TIM2 - DC2 (PA0, PA1)
///        TIM4 - DC1 (PB6, PB7)
static void _enc_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    tim_time_base_init_t tim_time_base_init_struct = {0};

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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

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
	tim_dc_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_TimeBaseInit(TIM1, &tim_time_base_init_struct);
    TIM_TimeBaseInit(TIM10, &tim_time_base_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_0_speed;
    TIM_OC1Init(TIM1, &tim_dc_oc_init_struct);
    TIM_OC2Init(TIM1, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_1_speed;
    TIM_OC3Init(TIM1, &tim_dc_oc_init_struct);
    TIM_OC4Init(TIM1, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_2_speed;
    TIM_OC1Init(TIM10, &tim_dc_oc_init_struct);
    TIM_OC2Init(TIM10, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_3_speed;
    TIM_OC3Init(TIM10, &tim_dc_oc_init_struct);
    TIM_OC4Init(TIM10, &tim_dc_oc_init_struct);

	_dc_stop();

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM10, ENABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM10, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM10, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM10, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, DISABLE);
	TIM_ARRPreloadConfig(TIM10, DISABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM10, ENABLE);
}

static void _pwm_for_dc_change_speed(void)
{
    TIM_CtrlPWMOutputs(LS_PWM_TIM, DISABLE);
    TIM_CtrlPWMOutputs(RS_PWM_TIM, DISABLE);

    LS_PWM_TIM->CH1CVR = dc_motor_set.dc_motor_speed_pwm[0];
    LS_PWM_TIM->CH2CVR = dc_motor_set.dc_motor_speed_pwm[0];

    LS_PWM_TIM->CH3CVR = dc_motor_set.dc_motor_speed_pwm[1];
    LS_PWM_TIM->CH4CVR = dc_motor_set.dc_motor_speed_pwm[1];

    RS_PWM_TIM->CH1CVR = dc_motor_set.dc_motor_speed_pwm[2];
    RS_PWM_TIM->CH2CVR = dc_motor_set.dc_motor_speed_pwm[2];

    RS_PWM_TIM->CH3CVR = dc_motor_set.dc_motor_speed_pwm[3];
    RS_PWM_TIM->CH4CVR = dc_motor_set.dc_motor_speed_pwm[3];

    TIM_CtrlPWMOutputs(LS_PWM_TIM, ENABLE);
    TIM_CtrlPWMOutputs(RS_PWM_TIM, ENABLE);
}

static void _get_speed_rpm(void)
{
    for (size_t dc_motor = RR_MOTOR; dc_motor < DC_MOTOR_NUMBER; dc_motor++)
    {
        int32_t ticks = 0;
        switch (dc_motor)
        {
        case RR_MOTOR:
            ticks = TIM4->CNT - ENC_COUNT_VALUE / 2;
            TIM4->CNT = ENC_COUNT_VALUE / 2;
            break;
        case FR_MOTOR:
            ticks = TIM9->CNT - ENC_COUNT_VALUE / 2;
            TIM9->CNT = ENC_COUNT_VALUE / 2;
            break;
        case RL_MOTOR:
            ticks = TIM5->CNT - ENC_COUNT_VALUE / 2;
            TIM5->CNT = ENC_COUNT_VALUE / 2;
            break;
        case FL_MOTOR:
            ticks = TIM2->CNT - ENC_COUNT_VALUE / 2;
            TIM2->CNT = ENC_COUNT_VALUE / 2;
            break;
        }
        if (ticks < 0)
        {
            ticks = -ticks;
        }

        xQueueSend(odometry_queue, (void*) &ticks, portMAX_DELAY);

        const double interval = 0.1;
        dc_motor_set.dc_motor_speed_rpm[dc_motor] = 60 * ticks / (ENC_TICS_ONE_WHEEL * interval);
    }
}

static void _pid_calculate(m_pid_s *pid)
{
    double input = 0;
    double error = 0;
    double d_input = 0;
    double output = 0;
    const double max_output_sum = 300;
    double interval = 0.1;

    input = pid->input;
    error = pid->target - abs(input);
    d_input = (input - pid->last_input) / interval;

    pid->output_sum += (error * interval);
    pid->output_sum = max(min(pid->output_sum, max_output_sum), -max_output_sum);

    output = (pid->kp * error) + (pid->ki * pid->output_sum) + (pid->kd * d_input);

    output = max(min(pid->out_max, output), pid->out_min);

    pid->output = output;
    pid->last_input = input;
}

//---------------------------------------------------- Functions -------------------------------------------------------

void dc_motor_driver_task(void *pvParameters)
{
    m_pid_s pid_0 = {.kp = 1.2, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 10, .output = 0,
                   .output_sum = 0, .target = dc_motor_set.target_speed_rpm[0]};
    m_pid_s pid_1 = {.kp = 1.2, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 10, .output = 0,
                   .output_sum = 0, .target = dc_motor_set.target_speed_rpm[0]};
    m_pid_s pid_2 = {.kp = 1.2, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 10, .output = 0,
                   .output_sum = 0, .target = dc_motor_set.target_speed_rpm[1]};
    m_pid_s pid_3 = {.kp = 1.2, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 10, .output = 0,
                   .output_sum = 0, .target = dc_motor_set.target_speed_rpm[1]};

	while(1)
	{
	    if (dc_motor_set.target_speed_rpm[LS] < 0 && dc_motor_set.target_speed_rpm[RS] < 0)
	    {
	        _dc_backward_run();
	    }
	    else if (dc_motor_set.target_speed_rpm[LS] > 0 && dc_motor_set.target_speed_rpm[RS] > 0)
	    {
            _dc_forward_run();
        }
	    else if (dc_motor_set.target_speed_rpm[LS] > 0 && dc_motor_set.target_speed_rpm[RS] < 0)
	    {
	        _dc_ls_forward_run();
	        _dc_rs_backward_run();
	    }
        else if (dc_motor_set.target_speed_rpm[LS] < 0 && dc_motor_set.target_speed_rpm[RS] > 0)
        {
            _dc_ls_backward_run();
            _dc_rs_forward_run();
        }
	    else
	    {
	        _dc_stop();
	        continue;
	    }

	    _get_speed_rpm();

	    pid_0.input = dc_motor_set.dc_motor_speed_rpm[0];
	    pid_0.target = abs(dc_motor_set.target_speed_rpm[0]);
	    _pid_calculate(&pid_0);
	    dc_motor_set.dc_motor_speed_pwm[0] = pid_0.output;

        pid_1.input = dc_motor_set.dc_motor_speed_rpm[1];
        pid_1.target = abs(dc_motor_set.target_speed_rpm[0]);
        _pid_calculate(&pid_1);
        dc_motor_set.dc_motor_speed_pwm[1] = pid_1.output;

        pid_2.input = dc_motor_set.dc_motor_speed_rpm[2];
        pid_2.target = abs(dc_motor_set.target_speed_rpm[1]);
        _pid_calculate(&pid_2);
        dc_motor_set.dc_motor_speed_pwm[2] = pid_2.output;

        pid_3.input = dc_motor_set.dc_motor_speed_rpm[3];
        pid_3.target = abs(dc_motor_set.target_speed_rpm[1]);
        _pid_calculate(&pid_3);
        dc_motor_set.dc_motor_speed_pwm[3] = pid_3.output;

	    _pwm_for_dc_change_speed();

	    vTaskDelay(PID_PERIOD_MS / portTICK_PERIOD_MS);
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
