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

#define PWM_MODE 					(TIM_OCMode_PWM1)

#define DC_MOTOR_NUMBER             (4)
#define LS                          (0)
#define RS                          (1)

#define MOTOR_MAX_VALUE				(254)
#define MOTOR_PWM_PRESCALER			(12)

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

#define LS_PWM_TIM                  (TIM1)
#define RS_PWM_TIM                  (TIM10)

#define LS_GPIO				        (GPIOA)
#define DC0_TIM1_CH1_OUTPUT_ENABLE  (TIM_CC1E)
#define DC0_TIM1_CH2_OUTPUT_ENABLE  (TIM_CC2E)
#define DC1_TIM1_CH3_OUTPUT_ENABLE  (TIM_CC3E)
#define DC1_TIM1_CH4_OUTPUT_ENABLE  (TIM_CC4E)

#define RS_GPIO				         (GPIOD)
#define DC2_TIM10_CH1_OUTPUT_ENABLE  (TIM_CC1E)
#define DC2_TIM10_CH2_OUTPUT_ENABLE  (TIM_CC2E)
#define DC3_TIM10_CH3_OUTPUT_ENABLE  (TIM_CC3E)
#define DC3_TIM10_CH4_OUTPUT_ENABLE  (TIM_CC4E)

#define ENC_NUM                     (4)
#define ENC_NUM_PER_DC              (2)

#define ENC_TIM2_TIM5_GPIO          (GPIOA)
#define ENC_TIM4_GPIO               (GPIOB)
#define ENC_TIM9_GPIO               (GPIOD)

#define RCC_TIM2_TIM5_GPIO          (RCC_APB2Periph_GPIOA)
#define RCC_TIM2_TIM4_GPIO          (RCC_APB2Periph_GPIOB)
#define RCC_TIM9_GPIO               (RCC_APB2Periph_GPIOD)

#define ENC_COUNT_VALUE             (0xffff)
#define ENC_TICS_ONE_WHEEL          (418)
#define ENC_PPR                     (7)

#define WHEEL_RADIUS_M             (0.022)
#define WHEEL_C_M                  (2 * PI * WHEEL_RADIUS_M)

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

typedef struct dc_motor_set
{
    double target_speed_rpm[DC_MOTOR_NUMBER / 2];

    uint32_t dc_motor_speed_pwm[DC_MOTOR_NUMBER];
    double dc_motor_speed_rpm[DC_MOTOR_NUMBER];
} dc_motor_set_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t dc_motor_driver_task_handler;
extern dc_motor_set_s dc_motor_set;

//------------------------------------------------ Function prototypes -------------------------------------------------

void dc_motor_driver_task_init(void);


#endif /* DC_MOTOR_DRIVER_H_ */
