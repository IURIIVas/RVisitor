/// \file blink_led.h
/// \brief Blink Led Header
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef TASKS_BLINK_LED_H_
#define TASKS_BLINK_LED_H_

//----------------------------------------------------- Includes -------------------------------------------------------

//------------------------------------------------------ Macros --------------------------------------------------------

#define LED1_BLINK_TASK_PRIO     	  (5)
#define LED1_BLINK_TASK_STK_SIZE      (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t blink_led1_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void led1_blink_task(void *pvParameters);
void blink_led_tasks_init(void);


#endif /* TASKS_BLINK_LED_H_ */
