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

#ifndef G_TESTBENCH
    #include "cmd_interface.h"
    #include "hw_201_survey.h"
    #include "hc_sr04_survey.h"
    #include "dc_motor_driver.h"
    #include "dc_motor_controller.h"
    #include "power_measure.h"
    #include "odometry.h"
#else
    #include "testbench.h"
#endif

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

#ifndef G_TESTBENCH

void tasks_init(void)
{
	cmd_iface_listening_task_init();
	hw_201_task_init();
    hc_sr04_task_init();
    power_measure_task_init();
    dc_motor_controller_task_init();
}

#endif

/// \brief Main Function
/// \param None
/// \return None
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();

#ifndef G_TESTBENCH
	tasks_init();

    vTaskStartScheduler();
#else
    testbench_init();
#endif

	while(1)
	{
	}
}
