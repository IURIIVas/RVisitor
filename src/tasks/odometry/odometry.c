/// \file odometry.c
/// \brief odometry algorithm source files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "odometry.h"
#include "dc_motor_controller.h"
#include "dc_motor_driver.h"
#include <math.h>

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t odometry_task_handler;

odometry_s odometry = {.x = 0, .y = 0, .theta = 0};
odometry_set_s odometry_set = {.cur_ticks = {0, 0, 0, 0}, .interval_s = 0.1};

QueueHandle_t odometry_queue;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

void odometry_task(void *pvParameters)
{
    double l_fr = 0;
    double l_fl = 0;
    double l_rr = 0;
    double l_rl = 0;
    double l = 0;
    double dtheta = 0;
    odometry_queue = xQueueCreate(DC_MOTOR_NUMBER, sizeof(int32_t));

    while(1)
    {
        xQueueReceive(odometry_queue, &(odometry_set.cur_ticks[RR_MOTOR]), portMAX_DELAY);
        xQueueReceive(odometry_queue, &(odometry_set.cur_ticks[FR_MOTOR]), portMAX_DELAY);
        xQueueReceive(odometry_queue, &(odometry_set.cur_ticks[RL_MOTOR]), portMAX_DELAY);
        xQueueReceive(odometry_queue, &(odometry_set.cur_ticks[FL_MOTOR]), portMAX_DELAY);

        l_rr = (double) (odometry_set.cur_ticks[RR_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
        l_fr = (double) (odometry_set.cur_ticks[FR_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
        l_rl = (double) (odometry_set.cur_ticks[RL_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
        l_fl = (double) (odometry_set.cur_ticks[FL_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;

        l = (((l_fl + l_rl) / 2) + ((l_fr + l_rr) / 2)) / 2;
        dtheta = (((l_fr + l_rr) / 2) - ((l_fl + l_rl) / 2)) / (2 * REF_POINT_DIST_M);

        odometry.x = odometry.x + (l*cos(odometry.theta + (dtheta / 2)));
        odometry.y = odometry.y + (l*sin(odometry.theta + (dtheta / 2)));
        odometry.theta = odometry.theta + dtheta;

        vTaskDelay(ODOMETRY_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/// \brief
/// \param
/// \retval
/// \return
void odometry_task_init(void)
{
    xTaskCreate((TaskFunction_t )odometry_task,
                (const char*    )"odometry algorithm",
                (uint16_t       )ODOMETRY_TASK_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )ODOMETRY_TASK_PRIORITY,
                (TaskHandle_t*  )&odometry_task_handler);
}
