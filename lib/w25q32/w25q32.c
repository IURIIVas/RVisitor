/// \file w24q32.c
/// \brief flash memory lib source file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//----------------------------------------------------- Includes -------------------------------------------------------

#include "w25q32.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------



//---------------------------------------------------- Functions -------------------------------------------------------

uint8_t spif_init(spif_s *spif)
{

}
//
//void spi_flash_init(spi_s *spi)
//{
//    spi_init_s spi_init_struct = {0};
//
//    spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    spi_init_struct.SPI_Mode = SPI_Mode_Master;
//    spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
//    spi_init_struct.SPI_CPOL = SPI_CPOL_High;
//    spi_init_struct.SPI_CPHA = SPI_CPHA_2Edge;
//    spi_init_struct.SPI_NSS = SPI_NSS_Soft;
//    spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//    spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
//    spi_init_struct.SPI_CRCPolynomial = 7;
//    SPI_Init(spi, &spi_init_struct);
//
//    SPI_Cmd(spi, ENABLE);
//}
//
//void spi_flash_write_enable(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port)
//{
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, w25_write_enable_cmd);
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//}
//
//void spi_flash_write_disable(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port)
//{
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, w25_write_disable_cmd);
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//}
//
//uint16_t spi_flash_read_id(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port)
//{
//    uint16_t byte = 0;
//
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, SPIF_MANUFACTUR_ID);
//    spi_readwrite_byte(spi, 0x00);
//    spi_readwrite_byte(spi, 0x00);
//    spi_readwrite_byte(spi, 0x00);
//    byte |= spi_readwrite_byte(spi, 0xFF) << 8;
//    byte |= spi_readwrite_byte(spi, 0xFF);
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//
//    return byte;
//}
//
///// \brief read w25q status register
/////        bit7  6   5   4   3   2   1   0
/////        spr   rv  tb  bp2 bp1 bp0 wel busy
///// \param spi
///// \retval uint8_t
///// \return status register
//uint8_t spi_flash_read_sr(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port)
//{
//    uint8_t sr = 0;
//
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, w25_read_status_reg_cmd);
//    sr = spi_readwrite_byte(spi, 0xFF);
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//
//    return sr;
//}
//
//uint8_t spi_flash_wait_busy(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port)
//{
//    uint8_t timeout = 200;
//
//    while ((spi_flash_read_sr(spi, cs_gpio, cs_port) & (1 << SR_BUSY_BIT)) && timeout)
//    {
//        timeout--;
//    }
//
//    return timeout;
//}
//
//uint8_t spi_flash_erase_sector(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port, uint32_t sect_numb)
//{
//    uint32_t addr = sect_numb * SECTOR_SIZE_BYTES;
//    uint8_t status = 0;
//
//    spi_flash_write_enable(spi, cs_gpio, cs_port);
//    status = spi_flash_wait_busy(spi, cs_gpio, cs_port);
//    if (status == FAIL)
//    {
//        return status;
//    }
//
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, w25_erase_sector_cmd);
//    spi_readwrite_byte(spi, (uint8_t) ((addr >> 16) && 0xFF));
//    spi_readwrite_byte(spi, (uint8_t) ((addr >> 8) && 0xFF));
//    spi_readwrite_byte(spi, (uint8_t) ((addr >> 0) && 0xFF));
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//    status = spi_flash_wait_busy(spi, cs_gpio, cs_port);
//    if (status == FAIL)
//    {
//        return status;
//    }
//}
//
//void spi_flash_read(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port,  uint8_t *read_buffer, uint32_t read_addr, uint32_t size)
//{
//    GPIO_WriteBit(cs_gpio, cs_port, 0);
//    spi_readwrite_byte(spi, w25_read_data_cmd);
//    spi_readwrite_byte(spi, (uint8_t) ((read_addr) >> 16) & 0xFF);
//    spi_readwrite_byte(spi, (uint8_t) ((read_addr) >> 8) & 0xFF);
//    spi_readwrite_byte(spi, (uint8_t) ((read_addr) >> 0) & 0xFF);
//
//    for (uint32_t idx = 0; idx < size; idx++)
//    {
//        read_buffer[idx] = spi_readwrite_byte(spi, 0xFF);
//    }
//
//    GPIO_WriteBit(cs_gpio, cs_port, 1);
//}
//
//void spi_flash_write(spi_s *spi, gpio_s *cs_gpio, uint8_t cs_port,  uint8_t *write_buffer, uint32_t write_addr, uint32_t size)
//{
//    uint32_t sector_pos = 0;
//    uint32_t sector_remain = 0;
//
//    sector_pos = write_addr / SECTOR_SIZE_BYTES;
//    sector_remain = SECTOR_SIZE_BYTES - (write_addr % SECTOR_SIZE_BYTES);
//
//    if (size <= sector_remain)
//    {
//        sector_remain = size;
//    }
//
//}
