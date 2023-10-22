/// \file m_string.c
/// \brief lib to help with strings in c
/// \author 1jura.vas@gmail.com
///
/// \details
///
//--------------------------------------------------- Include guards ---------------------------------------------------

//----------------------------------------------------- Includes -------------------------------------------------------

#include <stdint.h>
#include "m_string.h"

//------------------------------------------------------ Macros --------------------------------------------------------

//----------------------------------------------------- Typedefs -------------------------------------------------------

//---------------------------------------------------- Variables -------------------------------------------------------

//------------------------------------------------ Function prototypes -------------------------------------------------

//------------------------------------------------- Inline Functions ---------------------------------------------------

//------------------------------------------------- Static Functions ---------------------------------------------------

//---------------------------------------------------- Functions -------------------------------------------------------

/// \brief Reverse string
/// \param char* str - string to reverse
/// \retval None
/// \return None
void reverse(char *str)
{
	uint32_t i = 0;
	uint32_t j = 0;
	char tmp_char;

	for (j = m_strlen(str) - 1; i < j; i++, j--)
	{
		tmp_char = str[i];
		str[i] = str[j];
		str[j] = tmp_char;
	}
}

/// \brief Transform int to string
/// \param int32_t n - number to convert, char* str - string where to convert
/// \retval None
/// \return None
void m_itoa(int32_t n, char *str)
{
	uint32_t i = 0;
	int sign = n;

	if (sign < 0)
	{
		n = -n;
	}

	while (n > 0)
	{
		str[i] = (n % 10) + '0';
		n /= 10;
		i++;
	}
	if (sign < 0)
	{
		str[i] = '-';
		i++;
	}
	str[i] = '\0';

	reverse(str);
}

/// \brief Get length of string
/// \param char* str - string
/// \retval uint32_t
/// \return length of string
uint32_t m_strlen(const char *str)
{
	uint32_t count = 0;

	while (*str)
	{
		count++;
		str++;
	}

	return count;
}

/// \brief Compare two strings
/// \param char* str1 - first string; char* str2 - second string
/// \retval int8
/// \return 1 if x == y, else 0
int8_t m_strcmp(const char *str1, const char *str2)
{
	while (*str1)
	{
		if (*str1 != *str2)
		{
			return 0;
		}

		str1++;
		str2++;
	}
	return 1;
}
