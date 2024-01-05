/// \file power_measure.h
/// \brief current and voltage measure with ina226 header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef POWER_MEASURE_H_
#define POWER_MEASURE_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "queue.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define I2C_RCC_GPIO_PORT               (RCC_APB2Periph_GPIOB)
#define I2C_GPIO_PORT                   (GPIOB)
#define I2C_GPIO_SDA_PIN                (GPIO_Pin_11)
#define I2C_GPIO_SCL_PIN                (GPIO_Pin_10)

#define I2C_RCC                         (RCC_APB1Periph_I2C2)
#define I2C_POWER_MES                   (I2C2)

#define INA226_BAUDRATE                 (100000)
#define INA226_ADDR                     (0x80)

#define POWER_MEAS_TASK_DELAY_MS        (10)

#define POWER_MEASURE_TASK_PRIORITY     (5)
#define POWER_MEASURE_TASK_STK_SIZE     (256)


//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef struct
{
    double current;
    double voltage;
} ina226_measures_s;

//---------------------------------------------------- Variables -------------------------------------------------------

extern ina226_measures_s power_measures_struct;
extern TaskHandle_t power_measure_task_handler;
extern QueueHandle_t queue_power_measure;

//------------------------------------------------ Function prototypes -------------------------------------------------

void power_measure_task_init(void);


#endif /* POWER_MEASURE_H_ */
