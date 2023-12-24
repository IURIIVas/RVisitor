/// \file main.c
/// \brief Main file.
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "misc.h"
#include "system_ch32v30x.h"

#include "blink_led.h"
#include "cmd_interface.h"
#include "hw_201_survey.h"
#include "hc_sr04_survey.h"
#include "dc_motor_driver.h"
#include "dc_motor_controller.h"
#include "power_measure.h"
#include "odometry.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------


void tasks_init(void)
{
//	blink_led_tasks_init();
	cmd_iface_listening_task_init();
	hw_201_task_init();
	dc_motor_driver_task_init();
	odometry_task_init();
    dc_motor_controller_task_init();
    power_measure_task_init();
//	hc_sr04_task_init();
}


/// \brief Main Function
/// \param None
/// \retval None
/// \return None
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();

	tasks_init();

    vTaskStartScheduler();

	while(1)
	{
	}
}
