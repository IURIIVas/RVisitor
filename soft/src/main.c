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
	#include "motion_control.h"
    #include "cmd_interface.h"
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
    motion_control_task_init();
}

#endif

/// \brief Main Function
/// \param None
/// \return None
int main(void)
{

    nvic_priority_group_config(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();

#ifndef G_TESTBENCH
    tasks_init();

    vTaskStartScheduler();
#else
    testbench_init();
#endif

    while(1) {
	}
}
