/// \file
/// \brief
/// \author
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "hc_sr04_survey.h"
#include "hc_sr04.h"
#include "tim.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

TaskHandle_t hc_sr04_survey_task_handler;
uint8_t hc_sr04_sensors_distance[] = {0, 0, 0, 0};

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

/// \brief Choose sensor by external MUX
/// \param None
/// \retval None
/// \return None
static void _gpio_mux_select_init(void)
{
	gpio_init_t gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = HC_SR04_MUX_S0_GPIO_PIN | HC_SR04_MUX_S1_GPIO_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HC_SR04_GPIO_PORT, &gpio_init_struct);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void hc_sr04_survey_task(void *pvParameters)
{
	uint8_t sensor_mux_select = 0;

	while (1)
	{
		switch (sensor_mux_select)
		{
		case 0:
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 1:
			GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 2:
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 3:
			GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		default:
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
		}

		GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
		HC_SR04_TRIG_TIMER->CNT = 0;
		while (HC_SR04_TRIG_TIMER->CNT != 10) {};
        GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);

		vTaskDelay(HC_SR04_DELAY_TICKS);
	}
}

/// \brief
/// \param
/// \retval
/// \return
void hc_sr04_task_init(void)
{
	hc_sr04_rcc_gpio_clk_init(HC_SR04_RCC_GPIO);
	hc_sr04_trig_base_tim_clk_init(HC_SR04_TRIG_TIMER_RCC);
	hc_sr04_trig_gpio_init(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
	hc_sr04_trig_base_tim_init(HC_SR04_TRIG_TIMER);
	_gpio_mux_select_init();

    xTaskCreate((TaskFunction_t )hc_sr04_survey_task,
				(const char*    )"hc_sr04 survey task",
				(uint16_t       )HC_SR04_SURVEY_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )HC_SR04_SURVEY_TASK_PRIORITY,
				(TaskHandle_t*  )&hc_sr04_survey_task_handler);
}
