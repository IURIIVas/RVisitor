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
char received_cmd[50];

const char *help_cmd = "help";
const char *print_hello_cmd = "print_hello";
const char *hw_201_state_cmd = "hw-201_state";
const char *dc0_motor_forward_cmd = "dc0_forward";
const char *dc0_motor_backward_cmd = "dc0_backward";
const char *dc0_motor_stop_cmd = "dc0_stop";

TaskHandle_t cmd_interface_task_handler;

//------------------------------------------------ Function prototypes -------------------------------------------------

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

/// \brief Enable clock on UART and its GPIO
/// \param None
/// \retval None
/// \return None
static void _rcc_gpio_cmd_iface_clk_init(void)
{
	RCC_APB2PeriphClockCmd(CMD_IFACE_RCC_APB2_GPIO, ENABLE);
	RCC_APB1PeriphClockCmd(CMD_IFACE_RCC_APB1_UART, ENABLE);
}

/// \brief GPIO init in alt mode to CMD_UART
/// \param None
/// \retval None
/// \return None
static void _gpio_cmd_iface_uart_init(void)
{
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

static inline void _dc0_set_forward(void)
{
	uart_send_str(CMD_IFACE_UART, "DC0 FORWARD\n");
	dc0_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc0_set_direction[MOTOR_STATE_IDX_DIR] = FORWARD_DIRECTION;
}

static inline void _dc0_set_backward(void)
{
	uart_send_str(CMD_IFACE_UART, "DC0 BACKWARD\n");
	dc0_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc0_set_direction[MOTOR_STATE_IDX_DIR] = BACKWARD_DIRECTION;
}

static inline void _dc0_stop(void)
{
	uart_send_str(CMD_IFACE_UART, "DC0 STOP\n");
	dc0_set_direction[MOTOR_STATE_IDX_SET] = SET_STATE;
	dc0_set_direction[MOTOR_STATE_IDX_DIR] = MOTOR_STOP;
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
	if (m_strcmp(received_cmd, help_cmd))
	{
		;
	}
	else if (m_strcmp(received_cmd, print_hello_cmd))
	{
		_print_hello();
	}
	else if (m_strcmp(received_cmd, hw_201_state_cmd))
	{
		_print_hw_201_state();
	}
	else if (m_strcmp(received_cmd, dc0_motor_forward_cmd))
	{
		_dc0_set_forward();
	}
	else if (m_strcmp(received_cmd, dc0_motor_backward_cmd))
	{
		_dc0_set_backward();
	}
	else if (m_strcmp(received_cmd, dc0_motor_stop_cmd))
	{
		_dc0_stop();
	}
	else
	{
		_print_unknown_cmd();
	}

	is_read_complete = 0;
}


//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief USART2 Handler
/// \param None
/// \retval None
/// \return None
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(CMD_IFACE_UART, USART_IT_RXNE) != RESET)
    {
    	char tmp_char = USART_ReceiveData(USART2);
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
