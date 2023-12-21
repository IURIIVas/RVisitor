/// \file w25q32.h
/// \brief flash memory lib header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef W25Q32_H_
#define W25Q32_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "spi.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define SPIF_MANUFACTUR_ID_CMD                    (0x90)
#define w25_write_enable_cmd                      (0x06)
#define w25_write_disable_cmd                     (0x04)
#define w25_read_status_reg_cmd                   (0x05)
#define w25_erase_sector_cmd                      (0x20)
#define w25_read_data_cmd                         (0x03)

#define SR_BUSY_BIT                               (0)

#define SECTOR_SIZE_BYTES                         (4096)
#define PAGE_SIZE_BYTES                           (256)

//----------------------------------------------------- Typedefs -------------------------------------------------------

typedef enum
{
    SPIF_MANUFACTOR_ERROR = 0,
    SPIF_MANUFACTOR_WINBOND = 0xEF,
    SPIF_MANUFACTOR_ISSI = 0xD5,
    SPIF_MANUFACTOR_MICRON = 0x20,
    SPIF_MANUFACTOR_GIGADEVICE = 0xC8,
    SPIF_MANUFACTOR_MACRONIX = 0xC2,
    SPIF_MANUFACTOR_SPANSION = 0x01,
    SPIF_MANUFACTOR_AMIC = 0x37,
    SPIF_MANUFACTOR_SST = 0xBF,
    SPIF_MANUFACTOR_HYUNDAI = 0xAD,
    SPIF_MANUFACTOR_ATMEL = 0x1F,
    SPIF_MANUFACTOR_FUDAN = 0xA1,
    SPIF_MANUFACTOR_ESMT = 0x8C,
    SPIF_MANUFACTOR_INTEL = 0x89,
    SPIF_MANUFACTOR_SANYO = 0x62,
    SPIF_MANUFACTOR_FUJITSU = 0x04,
    SPIF_MANUFACTOR_EON = 0x1C,
    SPIF_MANUFACTOR_PUYA = 0x85,
} spif_manufactor_e;


typedef enum
{
    SPIF_SIZE_ERROR = 0,
    SPIF_SIZE_1MBIT = 0x11,
    SPIF_SIZE_2MBIT = 0x12,
    SPIF_SIZE_4MBIT = 0x13,
    SPIF_SIZE_8MBIT = 0x14,
    SPIF_SIZE_16MBIT = 0x15,
    SPIF_SIZE_32MBIT = 0x16,
    SPIF_SIZE_64MBIT = 0x17,
    SPIF_SIZE_128MBIT = 0x18,
    SPIF_SIZE_256MBIT = 0x19,
    SPIF_SIZE_512MBIT = 0x20,
} spif_size_e;

typedef struct
{
    spi_s *spi;
    gpio_s *cs_gpio;
    spif_manufactor_e manufactor;
    spif_size_e size;
    uint8_t cs_port;
    uint8_t lock;
    uint16_t reserved;
    uint32_t page_cnt;
    uint32_t sector_cnt;
    uint32_t block_cnt;
} spif_s;

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

uint8_t spif_init(spif_s *spif);

uint8_t spif_erase_chip(spif_s *spif);
uint8_t spif_erase_sector(spif_s *spif, uint32_t sect_num);
uint8_t spif_erase_block(spif_s *spif, uint32_t sect_num);

uint8_t spif_write_addr(spif_s *spif, uint32_t addr, uint8_t *data, uint32_t size);
uint8_t spif_write_page(spif_s *spif, uint32_t page_num, uint8_t *data, uint32_t size, uint32_t offset);
uint8_t spif_write_sector(spif_s *spif, uint32_t sector_num, uint8_t *data, uint32_t size, uint32_t offset);
uint8_t spif_write_block(spif_s *spif, uint32_t block_num, uint8_t *data, uint32_t size, uint32_t offset);

uint8_t spif_read_addr(spif_s *spif, uint32_t addr, uint8_t *data, uint32_t size);
uint8_t spif_read_page(spif_s *spif, uint32_t page_num, uint8_t *data, uint32_t size, uint32_t offset);
uint8_t spif_read_sector(spif_s *spif, uint32_t sector_num, uint8_t *data, uint32_t size, uint32_t offset);
uint8_t spif_read_block(spif_s *spif, uint32_t block_num, uint8_t *data, uint32_t size, uint32_t offset);

#endif /* W25Q32_H_ */
