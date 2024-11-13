/// \file hw_201_survey.c
/// \brief Survey of obstacles under wheels header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef HW_201_SURVEY_H_
#define W_201_SURVEY_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "queue.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define HW_201_SENSORS_NUM					  (4)

#define HW_201_GPIO_PORT					  (GPIOC)
#define HW_201_IN_GPIO_PIN					  (GPIO_PIN_2)
#define HW_201_MUX_GPIO_PINS				  (GPIO_PIN_0 | GPIO_PIN_1)
#define HW_201_RCC_GPIO						  (RCC_APB2Periph_GPIOC)

#define HW_201_SURVEY_TASK_PRIORITY     	  (5)
#define HW_201_SURVEY_TASK_STK_SIZE      	  (256)
#define HW_201_DELAY_MS 					  (10)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef enum {
	HW_201_FRONT_LEFT = 0,
	HW_201_FRONT_RIGHT = 1,
	HW_201_BACK_LEFT = 2,
	HW_201_BACK_RIGHT = 3
} HW_201_POS;

typedef enum {
	IS_OBSTACLE = 1,
	NO_OBSTACLE = 0
} HW_201_STATE;

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t hw_201_survey_task_handler;
extern uint8_t hw_201_sensors_state[];
extern QueueHandle_t queue_hw_201;

//------------------------------------------------ Function prototypes -------------------------------------------------

void hw_201_task_init(void);

#endif /* HW_201_SURVEY_H_ */
