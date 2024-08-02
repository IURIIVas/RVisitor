/// \file motion_control.h
/// \brief mmm header files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef DC_MOTOR_CONTROLLER_H_
#define DC_MOTOR_CONTROLLER_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "dc_motor_controller.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define DC_MOTOR_NUMBER                       (4)

#define PID_PERIOD_MS                         (100)

#define WHEEL_RADIUS_M                        (0.022)
#define WHEEL_C_M                             (2 * PI * WHEEL_RADIUS_M)

#define DC_MOTOR_CONTROLLER_TASK_PRIORITY     (5)
#define DC_MOTOR_CONTROLLER_TASK_STK_SIZE     (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t motion_control_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void motion_control_task_init(void);


#endif /* DC_MOTOR_CONTROLLER_H_ */
