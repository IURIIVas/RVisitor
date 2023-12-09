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

#include "m_string.h"
#include "cmd_interface.h"
#include "hw_201_survey.h"
#include "dc_motor_driver.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

uint8_t is_read_complete = 0;
uint8_t received_cmd_pos = 0;
char received_cmd[CMD_MAX_LEN];
int32_t subcmd[10];

const char *help_cmd = "help";
const char *print_hello_cmd = "print_hello";
const char *hw_201_state_cmd = "hw-201_state";
const char *dc_motor_forward_cmd = "dc_forward";
const char *dc_motor_backward_cmd = "dc_backward";
const char *dc_motor_left_cmd = "dc_left";
const char *dc_motor_right_cmd = "dc_right";
const char *dc_motor_stop_cmd = "dc_stop";
const char *dc_motor_set_speed = "dc_set_speed";

TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static int32_t _cmd_get_subcmd(const char *cmd)
{
	uint32_t idx_number = 0;
	int32_t number;

	while (cmd[idx_number] && cmd[idx_number] != ' ')
	{
		idx_number++;
	}

	number = m_strtol(cmd, idx_number);
	return number;
}

static void _cmd_parse_for_subcmd(const char *cmd)
{
	uint8_t subcmd_idx = 0;

	while (*cmd)
	{
		if (' ' == *cmd)
		{
			subcmd[subcmd_idx] = _cmd_get_subcmd(cmd + 1);
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

/// \brief Enable clock on UART and its GPIO
/// \param None
/// \retval None
/// \return None
static void _rcc_gpio_cmd_iface_clk_init(void)
{
	RCC_APB2PeriphClockCmd(CMD_IFACE_RCC_APB2_GPIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(CMD_IFACE_RCC_APB1_UART, ENABLE);
}

/// \brief GPIO init in alt mode to CMD_UART
/// \param None
/// \retval None
/// \return None
static void _gpio_cmd_iface_uart_init(void)
{
	AFIO->PCFR1 |= GPIO_Remap_USART2;

	gpio_init_t gpio_init_struct = {0};

	gpio_init_struct.GPIO_Pins = CMD_IFACE_GPIO_TX_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CMD_IFACE_GPIO, &gpio_init_struct);

	gpio_init_struct.GPIO_Pins = CMD_IFACE_GPIO_RX_PIN;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(CMD_IFACE_GPIO, &gpio_init_struct);
}

static void _nvic_enable_uart_interrupt(void)
{
	nvic_init_t nvic_init_struct = {0};

	nvic_init_struct.NVIC_IRQChannel = CMD_IFACE_NVIC_IRQ_CHANNEL;
	nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
	nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvic_init_struct);
}

/// \brief CMD_UART params init
/// \param None
/// \retval None
/// \return None
static void _uart_cmd_iface_init(void)
{
	uart_init_t uart_init_struct = {0};

	uart_init_struct.USART_BaudRate = CMD_IFACE_BAUDRATE;
	uart_init_struct.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
	uart_init_struct.USART_Parity = USART_Parity_No;
	uart_init_struct.USART_StopBits = USART_StopBits_1;
	uart_init_struct.USART_WordLength = USART_WordLength_8b;
	uart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	uart_init(CMD_IFACE_UART, &uart_init_struct);
	USART_ITConfig(CMD_IFACE_UART, USART_IT_RXNE, ENABLE);
}

/// \brief print hello world task
/// \param None
/// \retval None
/// \return None
static void _print_hello(void)
{
	const char *hello_string = "hello, world!\n";

	uart_send_str(CMD_IFACE_UART, hello_string);
}

static void _print_hw_201_state(void)
{
	uint8_t hw_201_sensor = HW_201_FRONT_LEFT;
	char *obstacle_found_state = "obstacle found !!\n";
	char *obstacle_not_found_state = "obstacle not found !!\n";

	for (hw_201_sensor = HW_201_FRONT_LEFT; hw_201_sensor < HW_201_SENSORS_NUM; hw_201_sensor++)
	{
		switch (hw_201_sensor)
		{
		case HW_201_FRONT_LEFT:
			uart_send_str(CMD_IFACE_UART, "front left sensor data: ");
			break;
		case HW_201_FRONT_RIGHT:
			uart_send_str(CMD_IFACE_UART, "front right sensor data: ");
			break;
		case HW_201_BACK_LEFT:
			uart_send_str(CMD_IFACE_UART, "back left sensor data: ");
			break;
		case HW_201_BACK_RIGHT:
			uart_send_str(CMD_IFACE_UART, "back right sensor data: ");
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

static void _dc_set_forward(void)
{
	uart_send_str(CMD_IFACE_UART, "DC FORWARD\n");
	dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_direction[MOTOR_STATE_IDX_DIR] = FORWARD_DIRECTION;
}

static void _dc_set_backward(void)
{
	uart_send_str(CMD_IFACE_UART, "DC BACKWARD\n");
	dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_direction[MOTOR_STATE_IDX_DIR] = BACKWARD_DIRECTION;
}

static void _dc_set_left(void)
{
	uart_send_str(CMD_IFACE_UART, "DC LEFT\n");
	dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_direction[MOTOR_STATE_IDX_DIR] = LEFT_DIRECTION;
}

static void _dc_set_right(void)
{
	uart_send_str(CMD_IFACE_UART, "DC RIGHT\n");
	dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_direction[MOTOR_STATE_IDX_DIR] = RIGHT_DIRECTION;
}

static void _dc_stop(void)
{
	uart_send_str(CMD_IFACE_UART, "DC STOP\n");
	dc_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_direction[MOTOR_STATE_IDX_DIR] = MOTOR_STOP;
}

static void _dc_set_state_speed(void)
{
	char speed_str[10];
	m_itoa(subcmd[0], speed_str);

	uart_send_str(CMD_IFACE_UART, "SET SPEED: ");
	uart_send_str(CMD_IFACE_UART, speed_str);
	uart_send_str(CMD_IFACE_UART, "\n");

	dc_set_speed[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc_set_speed[MOTOR_STATE_IDX_SPEED] = subcmd[0];
}

/// \brief print unknown cmd
/// \param None
/// \retval None
/// \return None
static void _print_unknown_cmd(void)
{
	const char *hello_string = "unknown cmd\n";

	uart_send_str(CMD_IFACE_UART, hello_string);
}

/// \brief Parse received CMD
/// \param received cmd
/// \retval None
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
	else if (m_strcmp(cmd_word, hw_201_state_cmd))
	{
		_print_hw_201_state();
	}
	else if (m_strcmp(cmd_word, dc_motor_forward_cmd))
	{
		_dc_set_forward();
	}
	else if (m_strcmp(cmd_word, dc_motor_backward_cmd))
	{
		_dc_set_backward();
	}
	else if (m_strcmp(cmd_word, dc_motor_left_cmd))
	{
		_dc_set_left();
	}
	else if (m_strcmp(cmd_word, dc_motor_right_cmd))
	{
		_dc_set_right();
	}
	else if (m_strcmp(cmd_word, dc_motor_stop_cmd))
	{
		_dc_stop();
	}
	else if (m_strcmp(cmd_word, dc_motor_set_speed))
	{
		_cmd_parse_for_subcmd(received_cmd);

		_dc_set_state_speed();
	}
	else
	{
		_print_unknown_cmd();
	}

	is_read_complete = 0;
}


//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief UART5 Handler
/// \param None
/// \retval None
/// \return None
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(CMD_IFACE_UART, USART_IT_RXNE) != RESET)
    {
    	char tmp_char = USART_ReceiveData(CMD_IFACE_UART);
        if('\n' == tmp_char)
        {
        	is_read_complete = 1;
        	received_cmd[received_cmd_pos] = '\0';
        	received_cmd_pos = 0;
        	return;
        }

        received_cmd[received_cmd_pos] = tmp_char;
        received_cmd_pos++;
    }
}

/// \brief UART Listening task
/// \param None
/// \retval None
/// \return None
void cmd_iface_listening_task(void *pvParameters)
{

    while(1)
    {
    	while (!is_read_complete)
    	{};

    	_cmd_parse();
    }
}

/// \brief Init listening to CMD UART iface
/// \param None
/// \retval None
/// \return None
void cmd_iface_listening_task_init(void)
{
	_rcc_gpio_cmd_iface_clk_init();
	_gpio_cmd_iface_uart_init();
	_uart_cmd_iface_init();
	_nvic_enable_uart_interrupt();
	USART_Cmd(CMD_IFACE_UART, ENABLE);					/// \todo: 妤快把快技快扼找我找抆 志 扳批扶抗扯我攻 uart_init

    xTaskCreate((TaskFunction_t )cmd_iface_listening_task,
				(const char*    )"cmd interface listening",
				(uint16_t       )CMD_IFACE_LISTENING_STK_SIZE,
				(void*          )NULL,
				(UBaseType_t    )CMD_IFACE_LISTENING_PRIO,
				(TaskHandle_t*  )&cmd_interface_task_handler);
}
