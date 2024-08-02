/// \file odometry.h
/// \brief odometry algorithm header files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef DC_MOTOR_CONTROLLER_H_
#define DC_MOTOR_CONTROLLER_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define DC_MOTOR_NUMBER                       (4)

#define PID_PERIOD_MS                         (100)

#define WHEEL_RADIUS_M                        (0.022)
#define WHEEL_C_M                             (2 * PI * WHEEL_RADIUS_M)

#define DC_MOTOR_CONTROLLER_TASK_PRIORITY     (5)
#define DC_MOTOR_CONTROLLER_TASK_STK_SIZE     (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct
{
    double kp;
    double ki;
    double kd;

    double input;
    uint32_t output;
    double target;

    double last_input;
    double output_sum;

    double out_min;
    double out_max;
} m_pid_s;

typedef struct
{
    double target_speed_rpm[DC_MOTOR_NUMBER / 2];
    double dc_motor_speed_rpm[DC_MOTOR_NUMBER];
    uint32_t dc_motor_speed_pwm[DC_MOTOR_NUMBER];


    uint8_t obstacle_flag;
    uint8_t no_surface_rear_flag;
    uint8_t no_surface_front_flag;
    uint8_t overcurrent_flag;
    uint8_t wheel_stuck_flag;
    uint8_t flags_enable;
} dc_motor_controller_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern dc_motor_controller_s dc_m_ctrl;

//------------------------------------------------ Function prototypes -------------------------------------------------

void dc_motors_speed_set(dc_motor_controller_s *dc_m_st, m_pid_s *pid_0,
						 m_pid_s *pid_1, m_pid_s *pid_2, m_pid_s *pid_3);

#endif /* DC_MOTOR_CONTROLLER_H_ */
