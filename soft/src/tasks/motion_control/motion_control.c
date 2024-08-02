/// \file dc_motor_controller.c
/// \brief set pwm connected to dc motors
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "odometry.h"
#include "dc_motor_controller.h"
#include "dc_motor_driver.h"
#include "encoder_driver.h"
#include "queue.h"
#include <math.h>

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t motion_control_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------


void motion_control_task(void *pvParameters)
{
	double kp = 1.5;
	double ki = 0.5;
	double kd = 0;

	dc_m_ctrl.target_speed_rpm[1] = 200;
	dc_m_ctrl.target_speed_rpm[0] = 200;

	m_pid_s pid_0 = {.kp = kp, .ki = ki, .kd = kd, .input = 0, .last_input = 0,
				   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
				   .output_sum = 0, .target = dc_m_ctrl.target_speed_rpm[0]};
	m_pid_s pid_1 = {.kp = kp, .ki = ki, .kd = kd, .input = 0, .last_input = 0,
				   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
				   .output_sum = 0, .target = dc_m_ctrl.target_speed_rpm[0]};
	m_pid_s pid_2 = {.kp = kp, .ki = ki, .kd = kd, .input = 0, .last_input = 0,
				   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
				   .output_sum = 0, .target = dc_m_ctrl.target_speed_rpm[1]};
	m_pid_s pid_3 = {.kp = kp, .ki = ki, .kd = kd, .input = 0, .last_input = 0,
				   .out_max = MOTOR_MAX_VALUE, .out_min = 1, .output = 0,
				   .output_sum = 0, .target = dc_m_ctrl.target_speed_rpm[1]};

	while(1) {
		for (size_t enc = RR_ENC; enc < ENC_NUM; enc++) {
			dc_m_ctrl.dc_motor_speed_rpm[enc] = get_speed_rpm(enc);
		}

		dc_motors_speed_set(&dc_m_ctrl, &pid_0, &pid_1, &pid_2, &pid_3);

		vTaskDelay(PID_PERIOD_MS / portTICK_PERIOD_MS);
	}
}

///// \brief DC motor controller task. PID controller and flags check
///// \param None
///// \return None
void motion_control_task_init(void)
{
    gpio_tim_dc_init();
    pwm_for_dc_init(0, 0, 0, 0);

    gpio_tim_enc_init();
    enc_init();

    xTaskCreate((TaskFunction_t )motion_control_task,
                (const char*    )"motion_control",
                (uint16_t       )DC_MOTOR_CONTROLLER_TASK_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )DC_MOTOR_CONTROLLER_TASK_PRIORITY,
                (TaskHandle_t*  )&motion_control_task_handler);
}
