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

#define LED1_BLINK_TASK_PRIO     	  5
#define LED1_BLINK_TASK_STK_SIZE      256
#define LED2_BLINK_TASK_PRIO     	  5
#define LED2_BLINK_TASK_STK_SIZE      256

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t blink_led1_task_handler;
TaskHandle_t blink_led2_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

/// \brief GPIO pins with LED on it initiation
/// \param None
/// \retval None
/// \return None
static void gpio_toggle_init(void)
{
  GPIO_InitTypeDef  GPIOA_init_struct = {0};
  GPIO_InitTypeDef  GPIOB_init_struct = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIOA_init_struct.GPIO_Pin = GPIO_Pin_15;
  GPIOA_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIOA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIOA_init_struct);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIOB_init_struct.GPIO_Pin = GPIO_Pin_4;
  GPIOB_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIOB_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIOB_init_struct);
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
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
        vTaskDelay(250);
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        vTaskDelay(250);
    }
}


/// \brief Blink LED2
/// \param None
/// \retval None
/// \return None
void led2_blink_task(void *pvParameters)
{
    while(1)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_4);
        vTaskDelay(500);
        GPIO_SetBits(GPIOB, GPIO_Pin_4);
        vTaskDelay(500);
    }
}

/// \brief Initiation of LED's blink tasks
/// \param None
/// \retval None
/// \return None
void blink_led_tasks_init(void)
{
	gpio_toggle_init();

    xTaskCreate((TaskFunction_t )led1_blink_task,
				(const char*    )"blink LED 1",
				(uint16_t       )LED1_BLINK_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )LED1_BLINK_TASK_PRIO,
				(TaskHandle_t*  )&blink_led1_task_handler);

    xTaskCreate((TaskFunction_t )led2_blink_task,
				(const char*    )"task2",
				(uint16_t       )LED2_BLINK_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )LED2_BLINK_TASK_PRIO,
				(TaskHandle_t*  )&blink_led2_task_handler);
}

