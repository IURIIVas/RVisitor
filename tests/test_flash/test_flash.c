/// \file test_flash.h
/// \brief test flash w25q32 source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include "test_flash.h"
#include "ch32v30x.h"
#include "spi.h"
#include "w25q32.h"
#include "uart.h"
#include "m_string.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

uint8_t rx_data[SECTOR_SIZE_BYTES];
uint8_t tx_data[SECTOR_SIZE_BYTES];

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _spi_flash_rcc_clk_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
}

static void _spi_flash_gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio_init_s spif_gpio = {0};
    spif_gpio.GPIO_Pins = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    spif_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    spif_gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &spif_gpio);

    spif_gpio.GPIO_Pins = GPIO_Pin_2;
    spif_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    spif_gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &spif_gpio);
}

static uint32_t _spif_id_check(spif_s *spif)
{
    uint32_t errors = 0;

    char flash_id[1] = "";
    m_itoa((uint8_t) spif->manufactor, flash_id);
    uart_send_str(CMD_UART, "SPIF ID: ");
    uart_send_str(CMD_UART, flash_id);
    uart_send_str(CMD_UART, "\n");

    if (spif->manufactor != 0xef)
    {
        uart_send_str(CMD_UART, "SPIF ID ERROR\n");
        errors++;
    }

    return errors;
}

static uint32_t _spif_sector_erase_check(spif_s *spif, uint32_t sector_num)
{
    uint32_t errors = 0;

    spif_erase_sector(spif, sector_num);
    spif_read_sector(spif, sector_num, rx_data, SECTOR_SIZE_BYTES, 0);
    for (uint32_t i = 0; i < SECTOR_SIZE_BYTES; i++)
    {
        if (rx_data[i] != 0xff)
        {
            uart_send_str(CMD_UART, "ERR IN BYTE: ");
            uart_send_int(CMD_UART, i);
            uart_send_str(CMD_UART, "\n");
            errors++;
        }
    }
    if (0 == errors)
    {
        uart_send_str(CMD_UART, "SECTOR ERASE OK\n");
    }

    return errors;
}

static uint32_t _spif_sector_readwrite_check(spif_s *spif, uint32_t sector_num)
{
    uint32_t errors = 0;

    spif_erase_sector(spif, sector_num);
    for (uint32_t i = 0; i < SECTOR_SIZE_BYTES; i++)
    {
        tx_data[i] = i;
    }

    spif_write_sector(spif, sector_num, tx_data, SECTOR_SIZE_BYTES, 0);
    spif_read_sector(spif, sector_num, rx_data, SECTOR_SIZE_BYTES, 0);

    for (uint32_t i = 0; i < SECTOR_SIZE_BYTES; i++)
    {
        if (rx_data[i] != tx_data[i])
        {
            uart_send_str(CMD_UART, "ERR IN BYTE: ");
            uart_send_int(CMD_UART, i);
            uart_send_str(CMD_UART, "\n");
            errors++;
        }
    }

    if (0 == errors)
    {
        uart_send_str(CMD_UART, "SECTOR READWRITE OK\n");
    }

    return errors;
}

//---------------------------------------------------- Functions -------------------------------------------------------

uint8_t w25q32_flash_test()
{
    _spi_flash_rcc_clk_init();
    _spi_flash_gpio_init();

    uint32_t errors = 0;
    spif_s spif;

    spif_init(&spif, SPI1, GPIOA, GPIO_Pin_2);

    errors += _spif_id_check(&spif);
    errors += _spif_sector_erase_check(&spif, 0);
    errors += _spif_sector_readwrite_check(&spif, 0);

    if (0 == errors)
    {
        uart_send_str(CMD_UART, "TEST PASS\n");
        return OK;
    }
    else
    {
        uart_send_str(CMD_UART, "TEST FAIL\n");
        return FAIL;
    }
}
