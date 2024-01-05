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

#define ENC_NUM                               (4)
#define ENC_NUM_PER_DC                        (2)

#define ENC_TIM2_TIM5_GPIO                    (GPIOA)
#define ENC_TIM4_GPIO                         (GPIOB)
#define ENC_TIM9_GPIO                         (GPIOD)

#define RCC_TIM2_TIM5_GPIO                    (RCC_APB2Periph_GPIOA)
#define RCC_TIM2_TIM4_GPIO                    (RCC_APB2Periph_GPIOB)
#define RCC_TIM9_GPIO                         (RCC_APB2Periph_GPIOD)

#define ENC_COUNT_VALUE                       (0xffff)
#define ENC_TICS_ONE_WHEEL                    (418)
#define ENC_PPR                               (7)

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
    double target_speed_rpm[ENC_NUM / 2];
    double dc_motor_speed_rpm[ENC_NUM];

    uint8_t obstacle_flag;
    uint8_t no_surface_rear_flag;
    uint8_t no_surface_front_flag;
    uint8_t overcurrent_flag;
    uint8_t wheel_stuck_flag;
    uint8_t flags_enable;
} dc_motor_controller_s;

typedef enum
{
    NO_STUCK = 0,
    STUCK = 1
} wheel_stuck_e;

typedef enum
{
    FRONT = 0,
    REAR = 1,
    LEFT = 2,
    RIGHT = 3
} obstacles_e;

//---------------------------------------------------- Variables -------------------------------------------------------

extern dc_motor_controller_s dc_motor_controller_struct;
extern TaskHandle_t dc_motor_controller_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void dc_motor_controller_task_init(void);


#endif /* DC_MOTOR_CONTROLLER_H_ */
