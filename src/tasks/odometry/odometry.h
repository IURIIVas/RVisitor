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

#define ODOMETRY_PERIOD_MS             (100)

#define ODOMETRY_TASK_PRIORITY         (5)
#define ODOMETRY_TASK_STK_SIZE         (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct odometry
{
    double x;
    double y;
    double theta;
} odometry_s;

typedef struct odometry_set
{
    uint32_t cur_ticks[DC_MOTOR_NUMBER_ODOMETRY];
    double interval_s;
} odometry_set_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t odometry_task_handler;
extern odometry_s odometry;
extern QueueHandle_t odometry_queue;

//------------------------------------------------ Function prototypes -------------------------------------------------

void odometry_task_init(void);


#endif /* ODOMETRY_H_ */
