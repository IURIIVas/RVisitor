/// \file odometry.c
/// \brief odometry algorithm source files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "odometry.h"
#include "dc_motor_controller.h"
#include "dc_motor_driver.h"
#include "hw_201_survey.h"
#include "hc_sr04_survey.h"
#include "power_measure.h"
#include "queue.h"
#include <math.h>

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t dc_motor_controller_task_handler;

dc_motor_controller_s dc_motor_controller_struct = {.dc_motor_speed_rpm = {0, 0, 0, 0}, .target_speed_rpm = {0, 0},
                                                    .obstacle_flag = 0, .overcurrent_flag = 0, .wheel_stuck_flag = 0,
                                                    .no_surface_rear_flag = 0, .no_surface_front_flag = 0,
                                                    .flags_enable = 0};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_tim_enc_init(void)
{
    AFIO->PCFR1 |= GPIO_FullRemap_TIM2;
    AFIO->PCFR2 |= GPIO_FullRemap_TIM9;
    gpio_init_s gpio_tim2_tim5_init_struct = {0};

    gpio_tim2_tim5_init_struct.GPIO_Pins = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_15;
    gpio_tim2_tim5_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim2_tim5_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_s gpio_tim2_tim4_init_struct = {0};

    gpio_tim2_tim4_init_struct.GPIO_Pins = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_3;
    gpio_tim2_tim4_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim2_tim4_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_init_s gpio_tim9_init_struct = {0};

    gpio_tim9_init_struct.GPIO_Pins = GPIO_Pin_9 | GPIO_Pin_11;
    gpio_tim9_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_tim9_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM5_GPIO, ENABLE);
    GPIO_Init(ENC_TIM2_TIM5_GPIO, &gpio_tim2_tim5_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM2_TIM4_GPIO, ENABLE);
    GPIO_Init(ENC_TIM4_GPIO, &gpio_tim2_tim4_init_struct);

    RCC_APB2PeriphClockCmd(RCC_TIM9_GPIO, ENABLE);
    GPIO_Init(ENC_TIM9_GPIO, &gpio_tim9_init_struct);
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
        dc_motor_controller_struct.dc_motor_speed_rpm[dc_motor] = 60 * ticks / (ENC_TICS_ONE_WHEEL * interval);
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

    pid->output = (uint32_t) output;
    pid->last_input = input;
}

static void _obstacle_flag_set_bit(uint16_t side, uint16_t bit)
{
    if (bit)
    {
        dc_motor_controller_struct.obstacle_flag |= (1 << side);
    }
    else
    {
        dc_motor_controller_struct.obstacle_flag &= ~(1 << side);
    }
}

static void _flags_get(void)
{
    uint8_t fl_obstacle = NO_OBSTACLE;
    uint8_t fr_obstacle = NO_OBSTACLE;
    uint8_t rl_obstacle = NO_OBSTACLE;
    uint8_t rr_obstacle = NO_OBSTACLE;

    xQueueReceive(queue_hw_201, &fl_obstacle, portMAX_DELAY);
    xQueueReceive(queue_hw_201, &fr_obstacle, portMAX_DELAY);
    xQueueReceive(queue_hw_201, &rl_obstacle, portMAX_DELAY);
    xQueueReceive(queue_hw_201, &rr_obstacle, portMAX_DELAY);

    if (IS_OBSTACLE == fl_obstacle || IS_OBSTACLE == fr_obstacle)
    {
        dc_motor_controller_struct.no_surface_front_flag = 1;
    }
    if (IS_OBSTACLE == rl_obstacle || IS_OBSTACLE == rr_obstacle)
    {
        dc_motor_controller_struct.no_surface_rear_flag = 1;
    }

    uint16_t front_dist = 0;
    uint16_t rear_dist = 0;
    uint16_t left_dist = 0;
    uint16_t right_dist = 0;

    xQueueReceive(queue_hc_sr04, &front_dist, portMAX_DELAY);
    xQueueReceive(queue_hc_sr04, &rear_dist, portMAX_DELAY);
    xQueueReceive(queue_hc_sr04, &left_dist, portMAX_DELAY);
    xQueueReceive(queue_hc_sr04, &right_dist, portMAX_DELAY);

    _obstacle_flag_set_bit((uint16_t) FRONT, front_dist);
    _obstacle_flag_set_bit((uint16_t) REAR, rear_dist);
    _obstacle_flag_set_bit((uint16_t) LEFT, left_dist);
    _obstacle_flag_set_bit((uint16_t) RIGHT, right_dist);

    double current = 0;
    double voltage = 0;

    xQueueReceive(queue_power_measure, &current, portMAX_DELAY);
    xQueueReceive(queue_power_measure, &voltage, portMAX_DELAY);
}

static uint32_t _flags_check(void)
{
    uint32_t direction = STOP;

    if (STUCK == dc_motor_controller_struct.wheel_stuck_flag)
    {
        direction = FORWARD_DIR;
    }
    if (dc_motor_controller_struct.overcurrent_flag)
    {
        return STOP;
    }
    if ((FORWARD_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << FRONT)))
    {
        return STOP;
    }
    if ((BACKWARD_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << REAR)))
    {
        return STOP;
    }
    if ((LEFT_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << LEFT)))
    {
        return STOP;
    }
    if ((RIGHT_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << RIGHT)))
    {
        return STOP;
    }
    if ((BACKWARD_DIR == direction) && dc_motor_controller_struct.no_surface_rear_flag)
    {
        return STOP;
    }
    if ((FORWARD_DIR == direction) && dc_motor_controller_struct.no_surface_front_flag)
    {
        return STOP;
    }

    return direction;
}

//---------------------------------------------------- Functions -------------------------------------------------------

void dc_motor_controller_task(void *pvParameters)
{
    m_pid_s pid_0 = {.kp = 0.5, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
                   .output_sum = 0, .target = dc_motor_controller_struct.target_speed_rpm[0]};
    m_pid_s pid_1 = {.kp = 0.5, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
                   .output_sum = 0, .target = dc_motor_controller_struct.target_speed_rpm[0]};
    m_pid_s pid_2 = {.kp = 0.5, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
                   .output_sum = 0, .target = dc_motor_controller_struct.target_speed_rpm[1]};
    m_pid_s pid_3 = {.kp = 0.5, .ki = 0.5, .kd = 0, .input = 0, .last_input = 0,
                   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
                   .output_sum = 0, .target = dc_motor_controller_struct.target_speed_rpm[1]};

    uint32_t direction = STOP;

    while(1)
    {
        if (dc_motor_controller_struct.target_speed_rpm[LS] < 0 && dc_motor_controller_struct.target_speed_rpm[RS] < 0)
        {
            direction = BACKWARD_DIR;
        }
        else if (dc_motor_controller_struct.target_speed_rpm[LS] > 0 && dc_motor_controller_struct.target_speed_rpm[RS] > 0)
        {
            direction = FORWARD_DIR;
        }
        else if (dc_motor_controller_struct.target_speed_rpm[LS] > 0 && dc_motor_controller_struct.target_speed_rpm[RS] < 0)
        {
            direction = LEFT_DIR;
        }
        else if (dc_motor_controller_struct.target_speed_rpm[LS] < 0 && dc_motor_controller_struct.target_speed_rpm[RS] > 0)
        {
            direction = RIGHT_DIR;
        }
        else
        {
            direction = STOP;
        }

        _get_speed_rpm();
        pid_0.input = dc_motor_controller_struct.dc_motor_speed_rpm[0];
        pid_1.input = dc_motor_controller_struct.dc_motor_speed_rpm[1];
        pid_2.input = dc_motor_controller_struct.dc_motor_speed_rpm[2];
        pid_3.input = dc_motor_controller_struct.dc_motor_speed_rpm[3];
        pid_0.target = abs(dc_motor_controller_struct.target_speed_rpm[0]);
        pid_1.target = abs(dc_motor_controller_struct.target_speed_rpm[0]);
        pid_2.target = abs(dc_motor_controller_struct.target_speed_rpm[1]);
        pid_3.target = abs(dc_motor_controller_struct.target_speed_rpm[1]);

        _pid_calculate(&pid_0);
        _pid_calculate(&pid_1);
        _pid_calculate(&pid_2);
        _pid_calculate(&pid_3);


        if (((0 == pid_0.input) || (0 == pid_1.input) || (0 == pid_2.input) || (0 == pid_3.input)) && direction != STOP)
        {
            dc_motor_controller_struct.wheel_stuck_flag = STUCK;
        }

        _flags_get();
        if (dc_motor_controller_struct.flags_enable)
        {
            direction = _flags_check();
        }

        xQueueSend(queue_dc_motor_driver, (void*) &direction, portMAX_DELAY);
        xQueueSend(queue_dc_motor_driver, (void*) &pid_0.output, portMAX_DELAY);
        xQueueSend(queue_dc_motor_driver, (void*) &pid_1.output, portMAX_DELAY);
        xQueueSend(queue_dc_motor_driver, (void*) &pid_2.output, portMAX_DELAY);
        xQueueSend(queue_dc_motor_driver, (void*) &pid_3.output, portMAX_DELAY);

        vTaskDelay(PID_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/// \brief DC motor controller task. PID controller and flags check
/// \param None
/// \return None
void dc_motor_controller_task_init(void)
{
    _gpio_tim_enc_init();
    _enc_init();

    xTaskCreate((TaskFunction_t )dc_motor_controller_task,
                (const char*    )"dc_motor_controller",
                (uint16_t       )DC_MOTOR_CONTROLLER_TASK_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DC_MOTOR_CONTROLLER_TASK_PRIORITY,
                (TaskHandle_t*  )&dc_motor_controller_task_handler);
}
