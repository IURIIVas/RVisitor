/// \file test_flash.h
/// \brief test flash w25q32 header file
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef TEST_FLASH_H_
#define TEST_FLASH_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define CMD_UART                (UART5)

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

uint8_t w25q32_flash_test();

#endif /* TEST_FLASH_H_ */
