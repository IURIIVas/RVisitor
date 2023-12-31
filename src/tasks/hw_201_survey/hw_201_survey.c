/// \file hw_201_survey.c
/// \brief Survey of obstacles under wheels source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "hw_201_survey.h"
#include "hw_201.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t hw_201_survey_task_handler;
uint8_t hw_201_sensors_state[] = {IS_OBSTACLE, IS_OBSTACLE, IS_OBSTACLE, IS_OBSTACLE};

QueueHandle_t queue_hw_201;

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_mux_select_init(void)
{
	gpio_init_s gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = HW_201_MUX_GPIO_PINS;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HW_201_GPIO_PORT, &gpio_init_struct);
}

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief check obstacles under the wheels task
/// \param None
/// \return None
void hw_201_survey_task(void *pvParameters)
{
	uint8_t sensor_mux_select = HW_201_FRONT_LEFT;
	uint8_t sensor_state = IS_OBSTACLE;
	queue_hw_201 = xQueueCreate(4, sizeof(uint8_t));

	while (1)
	{
		switch (sensor_mux_select)
		{
		case HW_201_FRONT_LEFT:
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_0);
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_1);
			break;
		case HW_201_FRONT_RIGHT:
			GPIO_SetBits(HW_201_GPIO_PORT, GPIO_Pin_0);
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_1);
			break;
		case HW_201_BACK_LEFT:
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_0);
			GPIO_SetBits(HW_201_GPIO_PORT, GPIO_Pin_1);
			break;
		case HW_201_BACK_RIGHT:
			GPIO_SetBits(HW_201_GPIO_PORT, GPIO_Pin_0);
			GPIO_SetBits(HW_201_GPIO_PORT, GPIO_Pin_1);
			break;
		default:
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_0);
			GPIO_ResetBits(HW_201_GPIO_PORT, GPIO_Pin_1);
		}

		sensor_state = hw_201_gpio_signal_in_get(HW_201_GPIO_PORT, HW_201_IN_GPIO_PIN);
		hw_201_sensors_state[sensor_mux_select] = sensor_state;

		sensor_mux_select = (sensor_mux_select + 1) % HW_201_SENSORS_NUM;

		xQueueSend(queue_hw_201, (void*) &sensor_state, portMAX_DELAY);
	}
}

/// \brief Init check obstacles under the wheels task
/// \param None
/// \return None
void hw_201_task_init(void)
{
	hw_201_rcc_gpio_clk_init(HW_201_RCC_GPIO);
	hw_201_gpio_signal_in_init(HW_201_GPIO_PORT, HW_201_IN_GPIO_PIN);
	_gpio_mux_select_init();

    xTaskCreate((TaskFunction_t )hw_201_survey_task,
				(const char*    )"hw_201 sensors get signal",
				(uint16_t       )HW_201_SURVEY_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )HW_201_SURVEY_TASK_PRIORITY,
				(TaskHandle_t*  )&hw_201_survey_task_handler);
}
