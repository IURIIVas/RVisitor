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

static inline uint32_t _spif_page_to_addr(uint32_t page_num)
{
    return page_num * PAGE_SIZE_BYTES;
}

static inline uint32_t _spif_sector_to_addr(uint32_t sector_num)
{
    return sector_num * SECTOR_SIZE_BYTES;
}

static inline uint32_t _spif_block_to_addr(uint32_t block_num)
{
    return block_num * BLOCK_SIZE_BYTES;
}

//------------------------------------------------- Static Functions ---------------------------------------------------

static void _spif_transmit_receive(spif_s *spif, uint8_t *tx, uint8_t *rx, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        rx[i] = spi_readwrite_byte(spif->spi, tx[i]);
    }
}

static void _spif_cs_set(spif_s *spif, uint8_t state)
{
    GPIO_WriteBit(spif->cs_gpio, spif->cs_pin, state);
    for (uint8_t i = 0; i < 10; i++)
    {
        __asm__("nop");
    }
}

static void _spi_flash_init(spi_s *spi)
{
    spi_init_s spi_init_struct = {0};

    spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_init_struct.SPI_Mode = SPI_Mode_Master;
    spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
    spi_init_struct.SPI_CPOL = SPI_CPOL_High;
    spi_init_struct.SPI_CPHA = SPI_CPHA_2Edge;
    spi_init_struct.SPI_NSS = SPI_NSS_Soft;
    spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
    spi_init_struct.SPI_CRCPolynomial = 7;
    SPI_Init(spi, &spi_init_struct);

    SPI_Cmd(spi, ENABLE);
}

static void _spif_write_disable(spif_s *spif)
{
    _spif_cs_set(spif, 0);
    spi_readwrite_byte(spif->spi, WRITE_DISABLE_CMD);
    _spif_cs_set(spif, 1);
}

static void _spif_write_enable(spif_s *spif)
{
    _spif_cs_set(spif, 0);
    spi_readwrite_byte(spif->spi, WRITE_ENABLE_CMD);
    _spif_cs_set(spif, 1);
}

static void _spif_find_chip(spif_s *spif)
{
    _spif_cs_set(spif, 0);
    spi_readwrite_byte(spif->spi, SPIF_JEDEC_ID_CMD);
    spif->manufactor = spi_readwrite_byte(spif->spi, 0xFF);
    spif->mem_type = spi_readwrite_byte(spif->spi, 0xFF);
    spif->size = spi_readwrite_byte(spif->spi, 0xFF);

    switch (spif->size)
    {
    case SPIF_SIZE_1MBIT:
        spif->block_cnt = 2;
        break;
    case SPIF_SIZE_2MBIT:
        spif->block_cnt = 4;
        break;
    case SPIF_SIZE_4MBIT:
        spif->block_cnt = 8;
        break;
    case SPIF_SIZE_8MBIT:
        spif->block_cnt = 16;
        break;
    case SPIF_SIZE_16MBIT:
        spif->block_cnt = 32;
        break;
    case SPIF_SIZE_32MBIT:
        spif->block_cnt = 64;
        break;
    case SPIF_SIZE_64MBIT:
        spif->block_cnt = 128;
        break;
    case SPIF_SIZE_128MBIT:
        spif->block_cnt = 256;
        break;
    case SPIF_SIZE_256MBIT:
        spif->block_cnt = 512;
        break;
    case SPIF_SIZE_512MBIT:
        spif->block_cnt = 1024;
        break;
    default:
        spif->size = SPIF_SIZE_ERROR;
        break;
    }

    spif->sector_cnt = spif->block_cnt * 16;
    spif->page_cnt = (spif->sector_cnt * SECTOR_SIZE_BYTES) / PAGE_SIZE_BYTES;
}

static uint8_t _spif_write_fn(spif_s *spif, uint32_t page_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t addr = 0;
    uint32_t maximum = PAGE_SIZE_BYTES - offset;
    uint8_t tx[5];

    if (page_num >= spif->page_cnt)
    {
        return FAIL;
    }
    if (offset >= PAGE_SIZE_BYTES)
    {
        return FAIL;
    }
    if (size > maximum)
    {
        size = maximum;
    }

    addr = _spif_page_to_addr(page_num) + offset;
    _spif_write_enable(spif);
    _spif_cs_set(spif, 0);
    if (spif->block_cnt >= 512)
    {
       tx[0] = PAGE_PROG4ADD_CMD;
       tx[1] = (addr >> 24) & 0xFF;
       tx[2] = (addr >> 16) & 0xFF;
       tx[3] = (addr >> 8) & 0xFF;
       tx[4] = (addr >> 0) & 0xFF;
       _spif_transmit_receive(spif, tx, tx, 5);
    }
    else
    {
        tx[0] = PAGE_PROG3ADD_CMD;
        tx[2] = (addr >> 16) & 0xFF;
        tx[3] = (addr >> 8) & 0xFF;
        tx[4] = (addr >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 4);
    }
    _spif_transmit_receive(spif, data, data, size);
    _spif_cs_set(spif, 1);

    _spif_write_disable(spif);
    return OK;
}

static uint8_t _spif_read_fn(spif_s *spif, uint32_t addr, uint8_t *data, uint32_t size)
{
    uint8_t tx[5];

    _spif_cs_set(spif, 0);
    if (spif->block_cnt >= 512)
    {
        tx[0] = READ4ADD_CMD;
        tx[1] = (addr >> 24) & 0xFF;
        tx[2] = (addr >> 16) & 0xFF;
        tx[3] = (addr >> 8) & 0xFF;
        tx[4] = (addr >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 5);
    }
    else
    {
        tx[0] = READ3ADD_CMD;
        tx[2] = (addr >> 16) & 0xFF;
        tx[3] = (addr >> 8) & 0xFF;
        tx[4] = (addr >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 4);

    }
    _spif_transmit_receive(spif, data, data, size);
    _spif_cs_set(spif, 1);

    return OK;
}

//---------------------------------------------------- Functions -------------------------------------------------------

uint8_t spif_init(spif_s *spif, spi_s *spi, gpio_s *cs_gpio, uint8_t cs_pin)
{
    if ((spif == NULL) || (spif->spi == NULL) || (spif->cs_gpio == NULL))
    {
        return FAIL;
    }

    spif->spi = spi;
    spif->cs_gpio = cs_gpio;
    spif->cs_pin = cs_pin;

    _spi_flash_init(spif->spi);
    _spif_cs_set(spif, 1);
    _spif_write_disable(spif);
    _spif_find_chip(spif);

    return OK;
}

void spif_erase_chip(spif_s *spif)
{
    _spif_cs_set(spif, 0);
    spi_readwrite_byte(spif->spi, SPIF_ERASE_CHIP_CMD);
    _spif_cs_set(spif, 1);

    _spif_write_disable(spif);
}

uint8_t spif_erase_sector(spif_s *spif, uint32_t sector)
{
    uint8_t tx[5];
    uint32_t address = sector * SECTOR_SIZE_BYTES;

    if (sector >= spif->sector_cnt)
    {
        return FAIL;
    }

    _spif_write_enable(spif);
    _spif_cs_set(spif, 0);
    if (spif->block_cnt >= 512)
    {
        tx[0] = SPIF_CMD_SECTOR_ERASE4ADD_CMD;
        tx[1] = (address >> 24) & 0xFF;
        tx[2] = (address >> 16) & 0xFF;
        tx[3] = (address >> 8) & 0xFF;
        tx[4] = (address >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 5);
    }
    else
    {
        tx[0] = SPIF_CMD_SECTOR_ERASE3ADD_CMD;
        tx[2] = (address >> 16) & 0xFF;
        tx[3] = (address >> 8) & 0xFF;
        tx[4] = (address >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 4);
    }
    _spif_cs_set(spif, 1);
    _spif_write_disable(spif);

    return OK;
}

uint8_t spif_erase_block(spif_s *spif, uint32_t block)
{
    uint8_t tx[5];
    uint32_t address = block * BLOCK_SIZE_BYTES;

    if (block >= spif->block_cnt)
    {
        return FAIL;
    }

    _spif_write_enable(spif);
    _spif_cs_set(spif, 0);
    if (spif->block_cnt >= 512)
    {
        tx[0] = SPIF_CMD_BLOCK_ERASE4ADD_CMD;
        tx[1] = (address >> 24) & 0xFF;
        tx[2] = (address >> 16) & 0xFF;
        tx[3] = (address >> 8) & 0xFF;
        tx[4] = (address >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 5);
    }
    else
    {
        tx[0] = SPIF_CMD_BLOCK_ERASE3ADD_CMD;
        tx[2] = (address >> 16) & 0xFF;
        tx[3] = (address >> 8) & 0xFF;
        tx[4] = (address >> 0) & 0xFF;
        _spif_transmit_receive(spif, tx, tx, 4);
    }
    _spif_cs_set(spif, 1);
    _spif_write_disable(spif);

    return OK;
}

uint16_t spif_read_id(spif_s *spif)
{
    uint16_t byte = 0;

    _spif_cs_set(spif, 0);
    spi_readwrite_byte(spif->spi, SPIF_MANUFACTUR_ID_CMD);
    spi_readwrite_byte(spif->spi, 0x00);
    spi_readwrite_byte(spif->spi, 0x00);
    spi_readwrite_byte(spif->spi, 0x00);
    byte |= spi_readwrite_byte(spif->spi, 0xFF) << 8;
    byte |= spi_readwrite_byte(spif->spi, 0xFF);
    _spif_cs_set(spif, 1);

    return byte;
}

uint8_t spif_write_addr(spif_s *spif, uint32_t addr, uint8_t *data, uint32_t size)
{
    uint32_t page = 0;
    uint32_t add = 0;
    uint32_t offset = 0;
    uint32_t remaining = 0;
    uint32_t length = 0;
    uint32_t index = 0;

    add = addr;
    remaining = size;

    do
    {
        page = _spif_page_to_addr(add);
        offset = add % PAGE_SIZE_BYTES;
        if (remaining <= PAGE_SIZE_BYTES)
        {
            length = remaining;
        }
        else
        {
            length = PAGE_SIZE_BYTES - offset;
        }
        _spif_write_fn(spif, page, &data[index], length, offset);
        add += length;
        index += length;
        remaining -= length;
        if (remaining == 0)
        {
            return OK;
        }

    } while(remaining > 0);

    return FAIL;
}

uint8_t spif_write_page(spif_s *spif, uint32_t page_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    return _spif_write_fn(spif, page_num, data, size, offset);
}

uint8_t spif_write_sector(spif_s *spif, uint32_t sector_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t bytes_written = 0;
    uint32_t page_num = 0;
    uint32_t remaining_bytes = 0;
    uint32_t page_offset = 0;
    uint32_t bytes_to_write = 0;

    if (offset >= SECTOR_SIZE_BYTES)
    {
        return FAIL;
    }
    if (size > (SECTOR_SIZE_BYTES - offset))
    {
        size = SECTOR_SIZE_BYTES - offset;
    }
    bytes_written = 0;
    page_num = sector_num * (SECTOR_SIZE_BYTES / PAGE_SIZE_BYTES);
    page_num += offset / PAGE_SIZE_BYTES;
    remaining_bytes = size;
    page_offset = offset % PAGE_SIZE_BYTES;
    while (remaining_bytes > 0 && page_num < ((sector_num + 1) * (SECTOR_SIZE_BYTES / PAGE_SIZE_BYTES)))
    {
        bytes_to_write = (remaining_bytes > PAGE_SIZE_BYTES) ? PAGE_SIZE_BYTES : remaining_bytes;
        if (_spif_write_fn(spif, page_num, data + bytes_written, bytes_to_write, page_offset) == FAIL)
        {
            return FAIL;
        }
        bytes_written += bytes_to_write;
        remaining_bytes -= bytes_to_write;
        page_num++;
        page_offset = 0;
    }

    return OK;
}

uint8_t spif_write_block(spif_s *spif, uint32_t block_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t bytes_written = 0;
    uint32_t page_num = 0;
    uint32_t remaining_bytes = 0;
    uint32_t page_offset = 0;
    uint32_t bytes_to_write = 0;

    if (offset >= BLOCK_SIZE_BYTES)
    {
        return FAIL;
    }
    if (size > (BLOCK_SIZE_BYTES - offset))
    {
        size = BLOCK_SIZE_BYTES - offset;
    }
    bytes_written = 0;
    page_num = block_num * (BLOCK_SIZE_BYTES / PAGE_SIZE_BYTES);
    page_num += offset / PAGE_SIZE_BYTES;
    remaining_bytes = size;
    page_offset = offset % PAGE_SIZE_BYTES;
    while (remaining_bytes > 0 && page_num < ((block_num + 1) * (BLOCK_SIZE_BYTES / PAGE_SIZE_BYTES)))
    {
        bytes_to_write = (remaining_bytes > PAGE_SIZE_BYTES) ? PAGE_SIZE_BYTES : remaining_bytes;
        if (_spif_write_fn(spif, page_num, data + bytes_written, bytes_to_write, page_offset) == FAIL)
        {
            return FAIL;
        }
        bytes_written += bytes_to_write;
        remaining_bytes -= bytes_to_write;
        page_num++;
        page_offset = 0;
    }

    return OK;
}

uint8_t spif_read_addr(spif_s *spif, uint32_t addr, uint8_t *data, uint32_t size)
{
    return _spif_read_fn(spif, addr, data, size);
}

uint8_t spif_read_page(spif_s *spif, uint32_t page_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t addr = _spif_page_to_addr(page_num);
    uint32_t maximum = PAGE_SIZE_BYTES - offset;
    if (size > maximum)
    {
        size = maximum;
    }
    return _spif_read_fn(spif, addr, data, size);
}

uint8_t spif_read_sector(spif_s *spif, uint32_t sector_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t addr = _spif_sector_to_addr(sector_num);
    uint32_t maximum = SECTOR_SIZE_BYTES - offset;
    if (size > maximum)
    {
        size = maximum;
    }
    return _spif_read_fn(spif, addr, data, size);
}

uint8_t spif_read_block(spif_s *spif, uint32_t block_num, uint8_t *data, uint32_t size, uint32_t offset)
{
    uint32_t addr = _spif_block_to_addr(block_num);
    uint32_t maximum = BLOCK_SIZE_BYTES - offset;
    if (size > maximum)
    {
        size = maximum;
    }
    return _spif_read_fn(spif, addr, data, size);
}
