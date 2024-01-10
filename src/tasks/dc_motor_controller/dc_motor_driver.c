/// \file dc_motor_driver.c
/// \brief DC_Motor driver source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "dc_motor_driver.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

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


//---------------------------------------------------- Functions -------------------------------------------------------

void gpio_tim_dc_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    AFIO->PCFR2 |= GPIO_FullRemap_TIM10;

    gpio_init_s gpio_left_side_init_struct = {0};

    gpio_left_side_init_struct.GPIO_Pins = GPIO_Pin_8 | GPIO_Pin_9
                                     | GPIO_Pin_10 | GPIO_Pin_11;
    gpio_left_side_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_left_side_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_s gpio_right_side_init_struct = {0};

    gpio_right_side_init_struct.GPIO_Pins = GPIO_Pin_1 | GPIO_Pin_3
                                     | GPIO_Pin_5 | GPIO_Pin_7;
    gpio_right_side_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_right_side_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init(LS_GPIO, &gpio_left_side_init_struct);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_Init(RS_GPIO, &gpio_right_side_init_struct);
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
/// \return None
void pwm_for_dc_init(uint32_t dc_0_speed, uint32_t dc_1_speed,
                             uint32_t dc_2_speed, uint32_t dc_3_speed)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    uint32_t period = MOTOR_MAX_VALUE;
    uint32_t prescaler = MOTOR_PWM_PRESCALER - 1;

    tim_time_base_init_s tim_time_base_init_struct = {0};

    tim_time_base_init_struct.TIM_Period = period;
    tim_time_base_init_struct.TIM_Prescaler = prescaler;
    tim_time_base_init_struct.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_time_base_init_struct.TIM_CounterMode = TIM_CounterMode_Up;

    tim_oc_init_t tim_dc_oc_init_struct = {0};

    tim_dc_oc_init_struct.TIM_OCMode = PWM_MODE;
    tim_dc_oc_init_struct.TIM_OutputState = TIM_OutputState_Enable;
    tim_dc_oc_init_struct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_TimeBaseInit(LS_PWM_TIM, &tim_time_base_init_struct);
    TIM_TimeBaseInit(RS_PWM_TIM, &tim_time_base_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_0_speed;
    TIM_OC1Init(LS_PWM_TIM, &tim_dc_oc_init_struct);
    TIM_OC2Init(LS_PWM_TIM, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_1_speed;
    TIM_OC3Init(LS_PWM_TIM, &tim_dc_oc_init_struct);
    TIM_OC4Init(LS_PWM_TIM, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_2_speed;
    TIM_OC1Init(RS_PWM_TIM, &tim_dc_oc_init_struct);
    TIM_OC2Init(RS_PWM_TIM, &tim_dc_oc_init_struct);

    tim_dc_oc_init_struct.TIM_Pulse = dc_3_speed;
    TIM_OC3Init(RS_PWM_TIM, &tim_dc_oc_init_struct);
    TIM_OC4Init(RS_PWM_TIM, &tim_dc_oc_init_struct);

    _dc_stop();

    TIM_CtrlPWMOutputs(LS_PWM_TIM, ENABLE);
    TIM_CtrlPWMOutputs(RS_PWM_TIM, ENABLE);

    TIM_OC1PreloadConfig(LS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(LS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(LS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(LS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC1PreloadConfig(RS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(RS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(RS_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(RS_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(LS_PWM_TIM, DISABLE);
    TIM_ARRPreloadConfig(RS_PWM_TIM, DISABLE);

    TIM_Cmd(LS_PWM_TIM, ENABLE);
    TIM_Cmd(RS_PWM_TIM, ENABLE);
}

void pwm_for_dc_change_speed(uint32_t *dc_motor_speed_pwm)
{
    TIM_CtrlPWMOutputs(LS_PWM_TIM, DISABLE);
    TIM_CtrlPWMOutputs(RS_PWM_TIM, DISABLE);

    LS_PWM_TIM->CH1CVR = dc_motor_speed_pwm[0];
    LS_PWM_TIM->CH2CVR = dc_motor_speed_pwm[0];

    LS_PWM_TIM->CH3CVR = dc_motor_speed_pwm[1];
    LS_PWM_TIM->CH4CVR = dc_motor_speed_pwm[1];

    RS_PWM_TIM->CH1CVR = dc_motor_speed_pwm[3];
    RS_PWM_TIM->CH2CVR = dc_motor_speed_pwm[3];

    RS_PWM_TIM->CH3CVR = dc_motor_speed_pwm[2];
    RS_PWM_TIM->CH4CVR = dc_motor_speed_pwm[2];

    TIM_CtrlPWMOutputs(LS_PWM_TIM, ENABLE);
    TIM_CtrlPWMOutputs(RS_PWM_TIM, ENABLE);
}

void direction_change(uint32_t direction)
{
    switch (direction)
    {
    case STOP:
        _dc_stop();
        break;
    case FORWARD_DIR:
        _dc_forward_run();
        break;
    case BACKWARD_DIR:
        _dc_backward_run();
        break;
    case LEFT_DIR:
        _dc_ls_forward_run();
        _dc_rs_backward_run();
        break;
    case RIGHT_DIR:
        _dc_ls_backward_run();
        _dc_rs_forward_run();
        break;
    default:
        _dc_stop();
        break;
    }
}
