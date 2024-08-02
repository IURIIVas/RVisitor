/// \file dc_motor_controller.c
/// \brief set pwm connected to dc motors
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "dc_motor_controller.h"
#include "dc_motor_driver.h"
#include "queue.h"
#include <math.h>

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

dc_motor_controller_s dc_m_ctrl = {.dc_motor_speed_rpm = {0, 0, 0, 0}, .target_speed_rpm = {0, 0},
								   .dc_motor_speed_pwm = {0, 0, 0, 0}, .obstacle_flag = 0,
								   .overcurrent_flag = 0, .wheel_stuck_flag = 0,
								   .no_surface_rear_flag = 0, .no_surface_front_flag = 0,
								   .flags_enable = 0};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------



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

//static void _obstacle_flag_set_bit(uint16_t side, uint16_t bit)
//{
//    if (bit)
//    {
//        dc_motor_controller_struct.obstacle_flag |= (1 << side);
//    }
//    else
//    {
//        dc_motor_controller_struct.obstacle_flag &= ~(1 << side);
//    }
//}

//static void _flags_get(void)
//{
//    uint8_t fl_obstacle = NO_OBSTACLE;
//    uint8_t fr_obstacle = NO_OBSTACLE;
//    uint8_t rl_obstacle = NO_OBSTACLE;
//    uint8_t rr_obstacle = NO_OBSTACLE;
//
//    xQueueReceive(queue_hw_201, &fl_obstacle, portMAX_DELAY);
//    xQueueReceive(queue_hw_201, &fr_obstacle, portMAX_DELAY);
//    xQueueReceive(queue_hw_201, &rl_obstacle, portMAX_DELAY);
//    xQueueReceive(queue_hw_201, &rr_obstacle, portMAX_DELAY);
//
//    if (IS_OBSTACLE == fl_obstacle || IS_OBSTACLE == fr_obstacle)
//    {
//        dc_motor_controller_struct.no_surface_front_flag = 1;
//    }
//    if (IS_OBSTACLE == rl_obstacle || IS_OBSTACLE == rr_obstacle)
//    {
//        dc_motor_controller_struct.no_surface_rear_flag = 1;
//    }
//
//    uint16_t front_dist = 0;
//    uint16_t rear_dist = 0;
//    uint16_t left_dist = 0;
//    uint16_t right_dist = 0;
//
//    xQueueReceive(queue_hc_sr04, &front_dist, portMAX_DELAY);
//    xQueueReceive(queue_hc_sr04, &rear_dist, portMAX_DELAY);
//    xQueueReceive(queue_hc_sr04, &left_dist, portMAX_DELAY);
//    xQueueReceive(queue_hc_sr04, &right_dist, portMAX_DELAY);
//
//    _obstacle_flag_set_bit((uint16_t) FRONT, front_dist);
//    _obstacle_flag_set_bit((uint16_t) REAR, rear_dist);
//    _obstacle_flag_set_bit((uint16_t) LEFT, left_dist);
//    _obstacle_flag_set_bit((uint16_t) RIGHT, right_dist);
//
//    double current = 0;
//    double voltage = 0;
//
//    xQueueReceive(queue_power_measure, &current, portMAX_DELAY);
//    xQueueReceive(queue_power_measure, &voltage, portMAX_DELAY);
//}

//static uint32_t _flags_check(void)
//{
//    uint32_t direction = STOP;
//
//    if (STUCK == dc_motor_controller_struct.wheel_stuck_flag)
//    {
//        direction = FORWARD_DIR;
//    }
//    if (dc_motor_controller_struct.overcurrent_flag)
//    {
//        return STOP;
//    }
//    if ((FORWARD_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << FRONT)))
//    {
//        return STOP;
//    }
//    if ((BACKWARD_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << REAR)))
//    {
//        return STOP;
//    }
//    if ((LEFT_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << LEFT)))
//    {
//        return STOP;
//    }
//    if ((RIGHT_DIR == direction) && (dc_motor_controller_struct.obstacle_flag & (1 << RIGHT)))
//    {
//        return STOP;
//    }
//    if ((BACKWARD_DIR == direction) && dc_motor_controller_struct.no_surface_rear_flag)
//    {
//        return STOP;
//    }
//    if ((FORWARD_DIR == direction) && dc_motor_controller_struct.no_surface_front_flag)
//    {
//        return STOP;
//    }
//
//    return direction;
//}

//---------------------------------------------------- Functions -------------------------------------------------------

///// \brief DC motor controller task. PID controller and flags check
///// \param None
///// \return None
void dc_motors_speed_set(dc_motor_controller_s *dc_m_st, m_pid_s *pid_0,
						 m_pid_s *pid_1, m_pid_s *pid_2, m_pid_s *pid_3)
{
	dc_motor_dir_e direction = STOP;

    if (dc_m_st->target_speed_rpm[LS] < 0 && dc_m_st->target_speed_rpm[RS] < 0)
    {
        direction = BACKWARD_DIR;
    }
    else if (dc_m_st->target_speed_rpm[LS] > 0 && dc_m_st->target_speed_rpm[RS] > 0)
    {
        direction = FORWARD_DIR;
    }
    else if (dc_m_st->target_speed_rpm[LS] > 0 && dc_m_st->target_speed_rpm[RS] < 0)
    {
        direction = LEFT_DIR;
    }
    else if (dc_m_st->target_speed_rpm[LS] < 0 && dc_m_st->target_speed_rpm[RS] > 0)
    {
        direction = RIGHT_DIR;
    }
    else
    {
        direction = STOP;
    }

    pid_0->input = dc_m_ctrl.dc_motor_speed_rpm[0];
    pid_1->input = dc_m_ctrl.dc_motor_speed_rpm[1];
    pid_2->input = dc_m_ctrl.dc_motor_speed_rpm[2];
    pid_3->input = dc_m_ctrl.dc_motor_speed_rpm[3];
    pid_0->target = abs(dc_m_ctrl.target_speed_rpm[0]);
    pid_1->target = abs(dc_m_ctrl.target_speed_rpm[0]);
    pid_2->target = abs(dc_m_ctrl.target_speed_rpm[1]);
    pid_3->target = abs(dc_m_ctrl.target_speed_rpm[1]);

    _pid_calculate(pid_0);
    _pid_calculate(pid_1);
    _pid_calculate(pid_2);
    _pid_calculate(pid_3);

    dc_m_ctrl.dc_motor_speed_pwm[0] = pid_0->output;
    dc_m_ctrl.dc_motor_speed_pwm[1] = pid_1->output;
    dc_m_ctrl.dc_motor_speed_pwm[2] = pid_2->output;
    dc_m_ctrl.dc_motor_speed_pwm[3] = pid_3->output;

    direction_change(direction);
    pwm_for_dc_change_speed(dc_m_ctrl.dc_motor_speed_pwm);
}

