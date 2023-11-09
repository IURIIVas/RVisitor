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
#include "dc_motor_driver.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------


void tasks_init(void)
{
	blink_led_tasks_init();
	cmd_iface_listening_task_init();
	hw_201_task_init();
	dc_motor_driver_task_init();
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
