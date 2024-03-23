/// \file blink_led.c
/// \brief Task to blink LED's.
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "blink_led.h"
#include "gpio.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t blink_led1_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

/// \brief GPIO pins with LED on it initiation
/// \param None
/// \retval None
/// \return None
static void _gpio_toggle_init(void)
{
  gpio_init_s  GPIOA_init_struct = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIOA_init_struct.gpio_pins = GPIO_PIN_15;
  GPIOA_init_struct.gpio_mode = GPIO_MODE_OUT_PP;
  GPIOA_init_struct.gpio_speed = GPIO_SPEED_50MHZ;
  gpio_init(GPIOA, &GPIOA_init_struct);
}

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief Blink LED1
/// \param None
/// \retval None
/// \return None
void led1_blink_task(void *pvParameters)
{
    while(1)
    {
        gpio_set_bits(GPIOA, GPIO_PIN_15);
        vTaskDelay(250);
        gpio_reset_bits(GPIOA, GPIO_PIN_15);
        vTaskDelay(250);
    }
}


/// \brief Initiation of LED's blink tasks
/// \param None
/// \retval None
/// \return None
void blink_led_tasks_init(void)
{
	_gpio_toggle_init();

    xTaskCreate((TaskFunction_t )led1_blink_task,
				(const char*    )"blink LED 1",
				(uint16_t       )LED1_BLINK_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )LED1_BLINK_TASK_PRIO,
				(TaskHandle_t*  )&blink_led1_task_handler);
}

