/// \file dc_motor_driver.h
/// \brief DC_Motor driver header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef DC_MOTOR_DRIVER_H_
#define DC_MOTOR_DRIVER_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define PWM_MODE 					(TIM_OCMode_PWM2)

#define MOTOR_MAX_VALUE				(1000)
#define MOTOR_PWM_PRESCALER			(1)

#define MOTOR_STATE_IDX_SET			(0)
#define MOTOR_STATE_IDX_DIR 		(1)
#define MOTOR_STATE_IDX_SPEED 		(1)

#define SET_STATE					(1)
#define STATE_SETTED				(0)

#define MOTOR_STOP					(0)
#define FORWARD_DIRECTION			(1)
#define BACKWARD_DIRECTION			(2)
#define LEFT_DIRECTION				(3)
#define RIGHT_DIRECTION				(4)

#define DC0_TIM1_GPIO				(GPIOA)
#define DC0_TIM1_CH1_OUTPUT_ENABLE  (TIM_CC1E)
#define DC0_TIM1_CH2_OUTPUT_ENABLE  (TIM_CC2E)

#define DC1_TIM1_GPIO				(GPIOA)
#define DC1_TIM1_CH3_OUTPUT_ENABLE  (TIM_CC3E)
#define DC1_TIM1_CH4_OUTPUT_ENABLE  (TIM_CC4E)

#define DC2_TIM8_GPIO				(GPIOC)
#define DC2_TIM8_CH1_OUTPUT_ENABLE  (TIM_CC1E)
#define DC2_TIM8_CH2_OUTPUT_ENABLE  (TIM_CC2E)

#define DC3_TIM8_GPIO				(GPIOC)
#define DC3_TIM8_CH3_OUTPUT_ENABLE  (TIM_CC3E)
#define DC3_TIM8_CH4_OUTPUT_ENABLE  (TIM_CC4E)

#define ENC_NUM                     (4)

#define ENC_TIM2_TIM3_TIM5_GPIO     (GPIOA)
#define ENC_TIM4_GPIO               (GPIOB)

#define ENC_COUNT_VALUE             (0xffff)
#define ENC_TICS_ONE_WHEEL          (418)

#define WHEEL_RADIUS_CM             (2.2)
#define WHEEL_C_CM                  (2 * PI * WHEEL_RADIUS_CM)

#define DC_MOTOR_DRIVER_TASK_PRIORITY     	  (5)
#define DC_MOTOR_DRIVER_TASK_STK_SIZE      	  (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct m_pid
{
    double kp;
    double ki;
    double kd;

    double input;
    double output;
    double target;

    double last_input;
    double output_sum;

    double out_min;
    double out_max;
} m_pid_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t dc_motor_driver_task_handler;
extern uint8_t dc0_set_direction[];
extern uint8_t dc_set_speed;
extern int32_t dc0_set_speed;
extern int32_t dc1_set_speed;
extern int32_t dc2_set_speed;
extern int32_t dc3_set_speed;
extern uint8_t dc_set_direction[];
extern double target_speed_lin_ms;

//------------------------------------------------ Function prototypes -------------------------------------------------

void dc_motor_driver_task_init(void);


#endif /* DC_MOTOR_DRIVER_H_ */
