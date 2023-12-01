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
uint8_t hc_sr04_sensors_distance[4] = {0, 0, 0, 0};
uint16_t hc_sr04_data_get = 0;

//------------------------------------------------ Function prototypes -------------------------------------------------

void TIM10_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

/// \brief CLK Enable to selected GPIO
/// \param APB Periph addr
/// \retval None
/// \return None
static inline void _hc_sr04_rcc_gpio_clk_init(void)
{
	RCC_APB2PeriphClockCmd(HC_SR04_RCC_GPIO, ENABLE);
}

/// \brief CLK Enable to selected timer
/// \param APB Periph addr
/// \retval None
/// \return None
static inline void _hc_sr04_trig_base_tim_clk_init(void)
{
	RCC_APB2PeriphClockCmd(HC_SR04_TRIG_TIMER_RCC, ENABLE);
}

/// \brief CLK Enable to selected timer
/// \param APB Periph addr
/// \retval None
/// \return None
static inline void _hc_sr04_echo_tim_clk_init(void)
{
	RCC_APB2PeriphClockCmd(HC_SR04_ECHO_TIMER_RCC, ENABLE);
}

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
		TIM_SetCounter(HC_SR04_TRIG_TIMER, 0);
		while (TIM_GetCounter(HC_SR04_TRIG_TIMER) != 10) {};
        GPIO_ResetBits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);

        vTaskDelay(100);

        hc_sr04_sensors_distance[sensor_mux_select] = hc_sr04_data_get;
        hc_sr04_data_get = 0;

        sensor_mux_select = (sensor_mux_select + 1) % HC_SR04_SENSORS_NUM;

		vTaskDelay(HC_SR04_DELAY_TICKS);
	}
}

void TIM10_CC_IRQHandler(void)
{
	uint16_t rising = TIM_GetCapture3(HC_SR04_ECHO_TIMER);
	uint16_t falling = TIM_GetCapture4(HC_SR04_ECHO_TIMER);

	uint16_t impulse_width = falling - rising;
	uint16_t distance_cm = impulse_width / 58;

	TIM_SetCounter(HC_SR04_ECHO_TIMER, 0);
	TIM_ClearITPendingBit(HC_SR04_ECHO_TIMER, TIM_IT_CC3 | TIM_IT_CC4);
	hc_sr04_data_get = distance_cm;
}

/// \brief
/// \param
/// \retval
/// \return
void hc_sr04_task_init(void)
{
	_hc_sr04_rcc_gpio_clk_init();
	_hc_sr04_trig_base_tim_clk_init();
	_hc_sr04_echo_tim_clk_init();
	hc_sr04_trig_gpio_init(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
	hc_sr04_trig_base_tim_init(HC_SR04_TRIG_TIMER);
	hc_sr04_echo_tim_init(HC_SR04_ECHO_TIMER);
	_gpio_mux_select_init();

    xTaskCreate((TaskFunction_t )hc_sr04_survey_task,
				(const char*    )"hc_sr04 survey task",
				(uint16_t       )HC_SR04_SURVEY_TASK_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )HC_SR04_SURVEY_TASK_PRIORITY,
				(TaskHandle_t*  )&hc_sr04_survey_task_handler);
}
