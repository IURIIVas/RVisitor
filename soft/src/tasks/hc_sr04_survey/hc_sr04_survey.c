/// \file hc_sr04_survey.c
/// \brief HC-SR04 ultrasonic distance sensor survey source file
/// \author 1jura.vas@gmail.com
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
uint16_t hc_sr04_sensors_distance[HC_SR04_SENSORS_NUM] = {0, 0, 0, 0};
uint16_t hc_sr04_data_get = 0;

QueueHandle_t queue_hc_sr04;

//------------------------------------------------ Function prototypes -------------------------------------------------

void TIM8_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

static inline void _hc_sr04_rcc_gpio_clk_init(void)
{
	RCC_APB2PeriphClockCmd(HC_SR04_RCC_GPIO, ENABLE);
}

static inline void _hc_sr04_trig_base_tim_clk_init(void)
{
	RCC_APB1PeriphClockCmd(HC_SR04_TRIG_TIMER_RCC, ENABLE);
}

static inline void _hc_sr04_echo_tim_clk_init(void)
{
	RCC_APB2PeriphClockCmd(HC_SR04_ECHO_TIMER_RCC, ENABLE);
}

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _gpio_mux_select_init(void)
{
	gpio_init_s gpio_init_struct = {0};

	gpio_init_struct.gpio_pins = HC_SR04_MUX_S0_GPIO_PIN | HC_SR04_MUX_S1_GPIO_PIN;
	gpio_init_struct.gpio_mode = GPIO_MODE_OUT_PP;
	gpio_init_struct.gpio_speed = GPIO_SPEED_50MHZ;
	gpio_init(HC_SR04_GPIO_PORT, &gpio_init_struct);
}

//---------------------------------------------------- Functions -------------------------------------------------------

void hc_sr04_survey_task(void *pvParameters)
{
    uint16_t warning = 0;
	uint8_t sensor_mux_select = 0;
	uint32_t timeout = 1000;

    queue_hc_sr04 = xQueueCreate(4, sizeof(uint16_t));

	while (1)
	{
	    timeout = 1000;
		switch (sensor_mux_select)
		{
		case 0:
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 1:
			gpio_set_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 2:
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			gpio_set_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		case 3:
			gpio_set_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			gpio_set_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
			break;
		default:
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S0_GPIO_PIN);
			gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_MUX_S1_GPIO_PIN);
		}

		gpio_set_bits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);
		TIM_SetCounter(HC_SR04_TRIG_TIMER, 0);
		while (TIM_GetCounter(HC_SR04_TRIG_TIMER) != 10 && timeout)
		{
		    timeout--;
		};
        gpio_reset_bits(HC_SR04_GPIO_PORT, HC_SR04_GPIO_TRIG_PIN);

        timeout = 1000;
        while (!hc_sr04_data_get && timeout)
        {
            timeout--;
        }

        hc_sr04_sensors_distance[sensor_mux_select] = hc_sr04_data_get;
        if (hc_sr04_data_get < DISTANCE_THRESHOLD_CM)
        {
            warning = 1;
        }
        else
        {
            warning = 0;
        }
        hc_sr04_data_get = 0;

        sensor_mux_select = (sensor_mux_select + 1) % HC_SR04_SENSORS_NUM;
        xQueueSend(queue_hc_sr04, (void*) &warning, portMAX_DELAY);
	}
	vTaskDelay(HC_SR04_DELAY_MS / portTICK_PERIOD_MS);
}

void TIM8_CC_IRQHandler(void)
{
	uint16_t rising = TIM_GetCapture3(HC_SR04_ECHO_TIMER);
	uint16_t falling = TIM_GetCapture4(HC_SR04_ECHO_TIMER);

	uint16_t impulse_width = falling - rising;
	uint16_t distance_cm = impulse_width / 58;

	TIM_SetCounter(HC_SR04_ECHO_TIMER, 0);
	TIM_ClearITPendingBit(HC_SR04_ECHO_TIMER, TIM_IT_CC3 | TIM_IT_CC4);
	hc_sr04_data_get = distance_cm;
}

/// \brief Distance sensors survey task.
/// \param None
/// \return None
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
