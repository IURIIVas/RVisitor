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

void TIM7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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

static void _nvic_enable_trig_tim_interrupt(void)
{
	nvic_init_t nvic_init_struct = {0};

	nvic_init_struct.NVIC_IRQChannel = HC_SR04_TRIG_TIMER_IRQ;
	nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvic_init_struct);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void TIM7_IRQHandler(void)
{
	GPIO_SetBits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
	TIM_ITConfig(HC_SR04_TRIG_TIMER, TIM_IT_Update, DISABLE);
}

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

		GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
		TIM_ITConfig(HC_SR04_TRIG_TIMER, TIM_IT_Update, ENABLE);
		while (!TIM_GetFlagStatus(HC_SR04_TRIG_TIMER, TIM_IT_Update))
		{
			__asm__("nop");
		}
		TIM_ClearFlag(HC_SR04_TRIG_TIMER, TIM_IT_Update);

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
	_nvic_enable_trig_tim_interrupt();
	_gpio_mux_select_init();

    xTaskCreate((TaskFunction_t )hc_sr04_survey_task,
				(const char*    )"hc_sr04 survey task",
				(uint16_t       )HC_SR04_SURVEY_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )HC_SR04_SURVEY_TASK_PRIORITY,
				(TaskHandle_t*  )&hc_sr04_survey_task_handler);
}
