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

odometry_s odometry = {.x = 0, .y = 0, .theta = 0};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief Odometry calculation with encoders.
/// \param None
/// \return None
void odometry_get(odometry_set_s *odometry_set)
{
    double l_fr = 0;
    double l_fl = 0;
    double l_rr = 0;
    double l_rl = 0;
    double l = 0;
    double dtheta = 0;

    l_rr = (double) (odometry_set->cur_ticks[RR_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
    l_fr = (double) (odometry_set->cur_ticks[FR_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
    l_rl = (double) (odometry_set->cur_ticks[RL_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;
    l_fl = (double) (odometry_set->cur_ticks[FL_MOTOR]) / ENC_TICS_ONE_WHEEL * WHEEL_C_M;

    double l_left_side = ((l_fl + l_rl) / 2);
    double l_right_side = ((l_fr + l_rr) / 2);
    if (abs(l_right_side - l_left_side) < 0.001)
    {
        l_left_side = l_right_side;
    }

    if (LEFT_DIR == odometry_set->direction)
    {
        l_left_side = -l_left_side;
    }
    if (RIGHT_DIR == odometry_set->direction)
    {
        l_right_side = -l_right_side;
    }
    l = (l_left_side + l_right_side) / 2;
    dtheta = (l_right_side - l_left_side) / (2 * REF_POINT_DIST_M);
    double x = (l*cos(odometry.theta + (dtheta / 2)));
    double y = (l*sin(odometry.theta + (dtheta / 2)));

    odometry.x = odometry.x + x;
    odometry.y = odometry.y + y;
    odometry.theta = odometry.theta + dtheta;
}
