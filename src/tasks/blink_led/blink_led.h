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

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t Task1Task_Handler;
extern TaskHandle_t Task2Task_Handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void led1_blink_task(void *pvParameters);
void led2_blink_task(void *pvParameters);
void blink_led_tasks_init(void);


#endif /* TASKS_BLINK_LED_H_ */
