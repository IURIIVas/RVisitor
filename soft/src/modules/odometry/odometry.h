/// \file odometry.h
/// \brief odometry algorithm header files
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "queue.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define DC_MOTOR_NUMBER_ODOMETRY       (4)

#define ECV_MOTOR_DIST_M               (0.12)
#define REF_POINT_DIST_M               (0.1)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct
{
    double x;
    double y;
    double theta;
} odometry_s;

typedef struct
{
    uint32_t cur_ticks[DC_MOTOR_NUMBER_ODOMETRY];
    double interval_s;
} odometry_set_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern odometry_s odometry;

//------------------------------------------------ Function prototypes -------------------------------------------------

void odometry_get(odometry_set_s *odometry_set);

#endif /* ODOMETRY_H_ */
