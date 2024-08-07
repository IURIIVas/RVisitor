/// \file cmd_interface.c
/// \brief UART command interface
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------


#include "global_inc.h"

#include "uart.h"
#include "gpio.h"
#include "rcc.h"
#include "spi.h"

#include "m_string.h"
#include "cmd_interface.h"
#include "hw_201_survey.h"
#include "hc_sr04_survey.h"
#include "dc_motor_controller.h"
#include "dc_motor_driver.h"
#include "power_measure.h"
#include "odometry.h"
#include "w25q32.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

uint8_t received_cmd_pos = 0;
char received_cmd[CMD_MAX_LEN];
double subcmd[10];

const char *help_cmd = "help";
const char *print_hello_cmd = "print_hello";
const char *get_obstacles = "get_obstacles";
const char *get_distances = "get_distances";
const char *get_measurements = "get_measurements";
const char *dc_motor_set_speed = "m";
const char *param_cliff_enable = "param_cliff_enable";
const char *param_obstacles_enable = "param_obstacles_enable";
const char *param_overcurrent_enable = "param_overcurrent_enable";
const char *param_speed_get_inf_enable = "param_speed_get_inf_enable";
const char *param_sens_survey_inf_enable = "param_sens_survey_inf_enable";
const char *param_flags_enable = "param_flags_enable";
const char *params_write = "params_write";
const char *params_read = "params_read";

module_params_s module_params = {.params = 0x1f};
spif_s spif = {.spi = SPI1, .cs_gpio = GPIOA, .cs_pin = GPIO_PIN_2};

TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void UART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static double _parse_str_for_number(const char *cmd)
{
	uint32_t idx_number = 0;
	double number;

	while (cmd[idx_number] && cmd[idx_number] != ' ')
	{
		idx_number++;
	}

	number = m_atof(cmd, idx_number);
	return number;
}

static void _cmd_parse_for_subcmd(const char *cmd)
{
	uint8_t subcmd_idx = 0;

	while (*cmd)
	{
		if (' ' == *cmd)
		{
			subcmd[subcmd_idx++] = _parse_str_for_number(cmd + 1);
		}
		cmd++;
	}
}

static void _cmd_parse_for_cmd(const char *cmd, char *cmd_word)
{
	uint32_t idx = 0;
	while (*cmd)
	{
		if (' ' == *cmd)
		{
			cmd_word[idx] = '\0';
			return;
		}
		cmd_word[idx] = *cmd;

		idx++;
		cmd++;
	}
	cmd_word[idx] = '\0';
}

static void _spi_flash_rcc_clk_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
}

static void _spi_flash_gpio_init(void)
{
    gpio_init_s spif_gpio = {0};
    spif_gpio.gpio_pins = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    spif_gpio.gpio_speed = GPIO_SPEED_50MHZ;
    spif_gpio.gpio_mode = GPIO_MODE_AF_PP;
    gpio_init(GPIOA, &spif_gpio);
}

static void _rcc_gpio_cmd_iface_clk_init(void)
{
	RCC_APB2PeriphClockCmd(CMD_IFACE_RCC_APB2_GPIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(CMD_IFACE_RCC_APB1_UART, ENABLE);
}

static void _gpio_cmd_iface_uart_init(void)
{
	AFIO->PCFR2 = (0x1 << 18);

	gpio_init_s gpio_init_struct = {0};

	gpio_init_struct.gpio_pins = CMD_IFACE_GPIO_TX_PIN;
	gpio_init_struct.gpio_mode = GPIO_MODE_AF_PP;
	gpio_init_struct.gpio_speed = GPIO_SPEED_50MHZ;
	gpio_init(CMD_IFACE_GPIO, &gpio_init_struct);

	gpio_init_struct.gpio_pins = CMD_IFACE_GPIO_RX_PIN;
	gpio_init_struct.gpio_mode = GPIO_MODE_IN_FLOATING;
	gpio_init(CMD_IFACE_GPIO, &gpio_init_struct);
}

static void _nvic_enable_uart_interrupt(void)
{
	nvic_init_s nvic_init_struct = {0};

	nvic_init_struct.NVIC_IRQChannel = CMD_IFACE_NVIC_IRQ_CHANNEL;
	nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvic_init_struct);
}

static void _uart_cmd_iface_init(void)
{
	uart_init_s uart_init_struct = {0};

	uart_init_struct.USART_BaudRate = CMD_IFACE_BAUDRATE;
	uart_init_struct.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
	uart_init_struct.USART_Parity = USART_Parity_No;
	uart_init_struct.USART_StopBits = USART_StopBits_1;
	uart_init_struct.USART_WordLength = USART_WordLength_8b;
	uart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	uart_init(CMD_IFACE_UART, &uart_init_struct);
	USART_ITConfig(CMD_IFACE_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(CMD_IFACE_UART, ENABLE);
}

static void _print_hello(void)
{
	const char *hello_string = "hello, world!\n";

	uart_send_str(CMD_IFACE_UART, hello_string);
}

static void _get_obstacles(void)
{
	uint8_t hw_201_sensor = HW_201_FRONT_LEFT;
	char *obstacle_found_state = "ok\n";
	char *obstacle_not_found_state = "warn\n";

	uart_send_str(CMD_IFACE_UART, "obstacles\n");
	for (hw_201_sensor = HW_201_FRONT_LEFT; hw_201_sensor < HW_201_SENSORS_NUM; hw_201_sensor++)
	{
		switch (hw_201_sensor)
		{
		case HW_201_FRONT_LEFT:
			uart_send_str(CMD_IFACE_UART, "fl: ");
			break;
		case HW_201_FRONT_RIGHT:
			uart_send_str(CMD_IFACE_UART, "fr: ");
			break;
		case HW_201_BACK_LEFT:
			uart_send_str(CMD_IFACE_UART, "rl: ");
			break;
		case HW_201_BACK_RIGHT:
			uart_send_str(CMD_IFACE_UART, "rr: ");
			break;
		default:
			uart_send_str(CMD_IFACE_UART, "YOU SHOUD NOT SEE THAT. ERR. No such sensor\n");
		}

		if (IS_OBSTACLE == hw_201_sensors_state[hw_201_sensor])
		{
			uart_send_str(CMD_IFACE_UART, obstacle_found_state);
		}
		else
		{
			uart_send_str(CMD_IFACE_UART, obstacle_not_found_state);
		}
	}
}

static void _dc_set_state_speed(void)
{
    char speed_str[10];
    double target_speed_lin_ms = subcmd[0];
    double target_speed_ang_rads = subcmd[1];

    ftoa(target_speed_lin_ms, speed_str, 3);
    uart_send_str(CMD_IFACE_UART, "LIN SPEED ");
    uart_send_str(CMD_IFACE_UART, speed_str);
    uart_send_str(CMD_IFACE_UART, " m/s\n");

    ftoa(target_speed_ang_rads, speed_str, 3);
    uart_send_str(CMD_IFACE_UART, "ANG SPEED ");
    uart_send_str(CMD_IFACE_UART, speed_str);
    uart_send_str(CMD_IFACE_UART, " rad/s\n");

    double target_speed_left_side_ms = target_speed_lin_ms + (target_speed_ang_rads * WHEEL_RADIUS_M);
    double target_speed_right_side_ms = target_speed_lin_ms - (target_speed_ang_rads * WHEEL_RADIUS_M);

    dc_m_ctrl.target_speed_rpm[LS] = (target_speed_left_side_ms * 60) / WHEEL_C_M;
    dc_m_ctrl.target_speed_rpm[RS] = (target_speed_right_side_ms * 60) / WHEEL_C_M;
}

static void _get_distances(void)
{
    uart_send_str(CMD_IFACE_UART, "distances\n");
    for (size_t hc_sr_num = 0; hc_sr_num < HC_SR04_SENSORS_NUM; hc_sr_num++)
    {
        uart_send_str(CMD_IFACE_UART, "sens ");
        uart_send_int(CMD_IFACE_UART, hc_sr_num);
        uart_send_str(CMD_IFACE_UART, ": ");
        uart_send_int(CMD_IFACE_UART, hc_sr04_sensors_distance[hc_sr_num]);
        uart_send_str(CMD_IFACE_UART, "\n");
    }
}

static void _get_current_and_voltage(void)
{
    uart_send_str(CMD_IFACE_UART, "current and voltage\n");
    uart_send_str(CMD_IFACE_UART, "current: ");
    uart_send_double(CMD_IFACE_UART, power_measures_struct.current);
    uart_send_str(CMD_IFACE_UART, "\n");
    uart_send_str(CMD_IFACE_UART, "voltage: ");
    uart_send_double(CMD_IFACE_UART, power_measures_struct.voltage);
    uart_send_str(CMD_IFACE_UART, "\n");
}

static void _params_set(uint8_t set, uint16_t bit)
{
    if (set)
    {
        module_params.params |= bit;
    }
    else
    {
        module_params.params &= ~(bit);
    }
}

static void _get_odometry(void)
{
    uart_send_str(CMD_IFACE_UART, "odometry\n");
    uart_send_str(CMD_IFACE_UART, "x: ");
    uart_send_double(CMD_IFACE_UART, odometry.x);
    uart_send_str(CMD_IFACE_UART, "\n");
    uart_send_str(CMD_IFACE_UART, "y: ");
    uart_send_double(CMD_IFACE_UART, odometry.y);
    uart_send_str(CMD_IFACE_UART, "\n");
    uart_send_str(CMD_IFACE_UART, "theta: ");
    uart_send_double(CMD_IFACE_UART, odometry.theta);
    uart_send_str(CMD_IFACE_UART, "\n");
}

static void _params_write(void)
{
    uart_send_str(CMD_IFACE_UART, "write parameters to flash\n");

    uint8_t params_write[2] = {module_params.params & 0xFF, (module_params.params >> 8) & 0xFF};
    uint8_t params_read[2];

    spif_erase_sector(&spif, 0);
    spif_write_addr(& spif, 0, params_write, 2);
    spif_read_addr(&spif, 0, params_read, 2);

    if ((params_read[0] != params_write[0]) || (params_read[1] != params_write[1]))
    {
        uart_send_str(CMD_IFACE_UART, "err while writing parameters\n");
    }

    uart_send_str(CMD_IFACE_UART, "read parameters from flash\n");
    uart_send_int(CMD_IFACE_UART, params_read[0]);
    uart_send_str(CMD_IFACE_UART, " ");
    uart_send_int(CMD_IFACE_UART, params_read[1]);
    uart_send_str(CMD_IFACE_UART, "\n");
}

static void _params_read(void)
{
    uart_send_str(CMD_IFACE_UART, "write parameters to flash\n");

    uint8_t params_read[2];

    spif_read_addr(&spif, 0, params_read, 2);

    uart_send_str(CMD_IFACE_UART, "read parameters from flash\n");
    uart_send_int(CMD_IFACE_UART, params_read[0]);
    uart_send_str(CMD_IFACE_UART, " ");
    uart_send_int(CMD_IFACE_UART, params_read[1]);
    uart_send_str(CMD_IFACE_UART, "\n");

    module_params.params = params_read[1] << 8 | params_read[0];
}

/// \brief print unknown cmd
/// \param None
/// \return None
static void _print_unknown_cmd(void)
{
	const char *hello_string = "unknown cmd\n";

	uart_send_str(CMD_IFACE_UART, hello_string);
}

/// \brief Parse received CMD
/// \param received cmd
/// \return None
static void _cmd_parse()
{
	char cmd_word[CMD_MAX_LEN];
	_cmd_parse_for_cmd(received_cmd, cmd_word);

	if (m_strcmp(cmd_word, help_cmd))
	{
		;
	}
	else if (m_strcmp(cmd_word, print_hello_cmd))
	{
		_print_hello();
	}
	else if (m_strcmp(cmd_word, get_obstacles))
	{
		_get_obstacles();
	}
	else if (m_strcmp(cmd_word, dc_motor_set_speed))
	{
		_cmd_parse_for_subcmd(received_cmd);
		_dc_set_state_speed();
	}
    else if (m_strcmp(cmd_word, get_distances))
    {
        _get_distances();
    }
    else if (m_strcmp(cmd_word, get_measurements))
    {
        _get_current_and_voltage();
    }
    else if (m_strcmp(cmd_word, params_write))
    {
        _params_write();
    }
    else if (m_strcmp(cmd_word, params_read))
    {
        _params_read();
    }
    else if (m_strcmp(cmd_word, param_cliff_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_CLIFF_ENABLE);
    }
    else if (m_strcmp(cmd_word, param_obstacles_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_OBSTACLES_ENABLE);
    }
    else if (m_strcmp(cmd_word, param_overcurrent_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_OVERCURRENT_ENABLE);
    }
    else if (m_strcmp(cmd_word, param_speed_get_inf_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_SPEED_GET_INF_ENABLE);
    }
    else if (m_strcmp(cmd_word, param_sens_survey_inf_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_SENS_SURVEY_INF_ENABLE);
    }
    else if (m_strcmp(cmd_word, param_flags_enable))
    {
        _cmd_parse_for_subcmd(received_cmd);
        _params_set((uint8_t)subcmd[0], PARAM_FLAGS_ENABLE);
        dc_m_ctrl.flags_enable = (uint8_t)subcmd[0];
    }
	else
	{
		_print_unknown_cmd();
	}
}


//---------------------------------------------------- Functions -------------------------------------------------------

void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(CMD_IFACE_UART, USART_IT_RXNE) != RESET)
    {
    	char tmp_char = USART_ReceiveData(CMD_IFACE_UART);
        if('\n' == tmp_char)
        {
        	received_cmd[received_cmd_pos] = '\0';
        	received_cmd_pos = 0;
        	_cmd_parse();
        	return;
        }

        received_cmd[received_cmd_pos] = tmp_char;
        received_cmd_pos++;
    }
}

/// \brief Serial port data print task
/// \param None
/// \return None
void cmd_iface_listening_task(void *pvParameters)
{

    while(1)
    {
        if (module_params.params & PARAM_SENS_SURVEY_INF_ENABLE)
        {
//            _get_obstacles();
//            _get_distances();
//            _get_odometry();
//            _get_current_and_voltage();

            vTaskDelay(CMD_INF_SURVEY_PERIOD_MS / portTICK_PERIOD_MS);
        }
    }
}

/// \brief Init listening to CMD UART iface
/// \param None
/// \return None
void cmd_iface_listening_task_init(void)
{
	_rcc_gpio_cmd_iface_clk_init();
	_gpio_cmd_iface_uart_init();
	_uart_cmd_iface_init();
	_nvic_enable_uart_interrupt();

    xTaskCreate((TaskFunction_t )cmd_iface_listening_task,
				(const char*    )"cmd interface listening",
				(uint16_t       )CMD_IFACE_LISTENING_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )CMD_IFACE_LISTENING_PRIO,
				(TaskHandle_t*  )&cmd_interface_task_handler);
}
