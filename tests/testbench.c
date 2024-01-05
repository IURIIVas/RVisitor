/// \file test_flash.h
/// \brief test flash w25q32 source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "testbench.h"
#include "ch32v30x.h"

#include "uart.h"
#include "gpio.h"
#include "rcc.h"
#include "spi.h"
#include "misc.h"

#include "m_string.h"

#include "test_flash.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

uint32_t test_num = 0;
char received_cmd[32];
uint8_t received_cmd_pos = 0;

//------------------------------------------------ Function prototypes -------------------------------------------------

void UART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

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

    gpio_init_struct.GPIO_Pins = CMD_IFACE_GPIO_TX_PIN;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CMD_IFACE_GPIO, &gpio_init_struct);

    gpio_init_struct.GPIO_Pins = CMD_IFACE_GPIO_RX_PIN;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CMD_IFACE_GPIO, &gpio_init_struct);
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

static void _print_tests(void)
{
    char test_num[32];

    m_itoa(FLASH_TEST, test_num);
    uart_send_str(CMD_IFACE_UART, "FLASH TEST: ");
    uart_send_str(CMD_IFACE_UART, test_num);
    uart_send_str(CMD_IFACE_UART, "\n");
}

static uint32_t _parse_str_for_number(const char *cmd)
{
    uint32_t idx_number = 0;
    uint32_t number;

    while (cmd[idx_number] && cmd[idx_number] != ' ')
    {
        idx_number++;
    }

    number = m_strtol(cmd, idx_number);
    return number;
}

static void _print_unknown_cmd(void)
{
    const char *hello_string = "unknown cmd\n";

    uart_send_str(CMD_IFACE_UART, hello_string);
}

static void _cmd_parse()
{
    test_num = _parse_str_for_number(received_cmd);
    if (test_num == FLASH_TEST)
    {
        uart_send_str(CMD_IFACE_UART, "w25q32 flash test\n");
        w25q32_flash_test();
    }
    else
    {
        _print_unknown_cmd();
    }
}

//---------------------------------------------------- Functions -------------------------------------------------------

void testbench_init(void)
{
    _rcc_gpio_cmd_iface_clk_init();
    _gpio_cmd_iface_uart_init();
    _uart_cmd_iface_init();
    _nvic_enable_uart_interrupt();
    USART_Cmd(CMD_IFACE_UART, ENABLE);
    _print_tests();

    w25q32_flash_test();
    while (1)
    {
        __asm__("nop");
    }
}

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
