/// \file m_string.h
/// \brief lib to help with strings in c
/// \author 1jura.vas@gmail.com
///
/// \details
///

//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef M_STRING_H_
#define M_STRING_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include <stdint.h>

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

int8_t m_strcmp(const char *x, const char *y);
uint32_t m_strlen(const char *str);
int32_t m_strtol(const char* str, uint32_t num_end_idx);
uint32_t m_itoa(int32_t n, char *str);
double m_atof(const char *arr);
void ftoa(double n, char* res, uint32_t afterpoint);
void reverse(char *str);

#endif /* M_STRING_H_ */
