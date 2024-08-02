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

#define PWM_MODE 					          (TIM_OCMode_PWM1)

#define MOTOR_MAX_VALUE				          (512)
#define MOTOR_PWM_PRESCALER			          (12)

#define LS_PWM_TIM                            (TIM1)
#define RS_PWM_TIM                            (TIM10)

#define LS_GPIO				                  (GPIOA)
#define DC0_TIM1_CH1_OUTPUT_ENABLE            (TIM_CC1E)
#define DC0_TIM1_CH2_OUTPUT_ENABLE            (TIM_CC2E)
#define DC1_TIM1_CH3_OUTPUT_ENABLE            (TIM_CC3E)
#define DC1_TIM1_CH4_OUTPUT_ENABLE            (TIM_CC4E)

#define RS_GPIO				                  (GPIOD)
#define DC2_TIM10_CH1_OUTPUT_ENABLE           (TIM_CC1E)
#define DC2_TIM10_CH2_OUTPUT_ENABLE           (TIM_CC2E)
#define DC3_TIM10_CH3_OUTPUT_ENABLE           (TIM_CC3E)
#define DC3_TIM10_CH4_OUTPUT_ENABLE           (TIM_CC4E)

#define DC_MOTOR_DRIVER_TASK_PRIORITY     	  (5)
#define DC_MOTOR_DRIVER_TASK_STK_SIZE      	  (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef enum
{
    RR_MOTOR = 0,
    FR_MOTOR = 1,
    FL_MOTOR = 2,
    RL_MOTOR = 3
} dc_motors_e;

typedef enum
{
    FORWARD_DIR = 0,
    BACKWARD_DIR = 1,
    LEFT_DIR = 2,
    RIGHT_DIR = 3,
    STOP = 4
} dc_motor_dir_e;

typedef enum
{
    LS = 0,
    RS = 1,
} dc_motor_side_e;

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

void pwm_for_dc_init(uint32_t dc_0_speed, uint32_t dc_1_speed, uint32_t dc_2_speed, uint32_t dc_3_speed);
void pwm_for_dc_change_speed(uint32_t *dc_motor_speed_pwm);
void direction_change(uint32_t direction);
void gpio_tim_dc_init(void);

#endif /* DC_MOTOR_DRIVER_H_ */
