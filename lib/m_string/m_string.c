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

/// \brief Convert string to int
/// \param cosnt char *str - pointer to a string
///        uint32_t num_end_idx - index where digits ended
/// \retval None
/// \return None
int32_t m_strtol(const char* str, uint32_t num_end_idx)
{
	uint32_t idx = 0;
	int32_t num = 0;
	uint8_t neg = 0;
	uint8_t base = 10;

	if (str[idx] == '-')
	{
		neg = 1;
		idx++;
	}

	while (idx != num_end_idx)
	{
		num *= base;
		num += str[idx] - '0';
		idx++;
	}
	if (neg)
	{
		num = -num;
	}
	return num;
}

/// \brief Convert int to string
/// \param int32_t n - number to convert
///        char* str - string where to convert
/// \retval None
/// \return None
uint32_t m_itoa(int32_t n, char *str)
{
	uint32_t i = 0;
	int sign = n;

	if (0 == n)
	{
	    str[0] = '0';
	    str[1] = '\0';
	    return 0;
	}

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
	return i;
}

/// \brief Get pow of a number
/// \params double n - nubmer
///         int32_t p - power
/// \retval double
/// \return powered value
double m_pow(double n, int32_t p)
{
    uint32_t i = 1;
    double m = n;

    if (p == 0)
    {
        return 1;
    }

    if (p > 0)
    {
        while (i != p)
        {
            m *= n;
            i++;
        }
    }
    else
    {
        while (i != p)
        {
            m /= n;
            i--;
        }
    }
    return m;
}

/// \brief String to double
/// \params char* arr - string
///         uint32_t num_end_idx - index where digits ended
/// \retval double
/// \return converted value
double m_atof(const char *str, uint32_t num_end_idx)
{
    double val = 0;
    uint32_t idx = 0;
    int32_t pow = 0;
    uint8_t neg = 0;
    uint8_t base = 10;
    uint8_t after_dot = 0;

    if (str[idx] == '-')
    {
        neg = 1;
        idx++;
    }

    while (idx != num_end_idx)
    {
        if (str[idx] != '.' && str[idx] != ',')
        {
            val *= base;
            val += str[idx] - '0';
            if (after_dot)
            {
                pow--;
            }
        }
        else
        {
            if (after_dot)
            {
                return 0;
            }
            after_dot = 1;
        }
        idx++;
    }
    val *= m_pow(10, pow);
    if (neg)
    {
        val = -val;
    }
    return val;
}

/// \brief Double number to string
/// \params char* res - string
///         double n - number to convert
///         uint32_t afterpoint - number of digits after point
/// \retval None
/// \return None
void ftoa(double n, char* res, uint32_t afterpoint)
{
    // Extract integer part
    int32_t ipart = (int32_t)n;

    // Extract floating part
    double fpart = n - (double)ipart;

    // convert integer part to string
    uint32_t i = m_itoa(ipart, res);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * m_pow(10, afterpoint);

        m_itoa((int32_t)fpart, res + i + 1);
    }
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
