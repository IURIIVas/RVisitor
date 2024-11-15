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
/// \return None
void reverse(char *str, int32_t len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

/// \brief Convert string to int
/// \param cosnt char *str - pointer to a string
///        uint32_t num_end_idx - index where digits ended
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
/// \return None
uint32_t m_itoa(int32_t n, char *str, int32_t d, uint8_t is_sign)
{
    uint32_t i = 0;
    int32_t sign = n;

    if (0 == n)
    {
        str[i] = '0';
        i++;
    }
    else
    {
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
        while (i < d)
        {
            str[i++] = '0';
        }
        if (is_sign)
        {
            if (sign < 0)
            {
                str[i] = '-';
                i++;
            }
        }
    }


    reverse(str, i);
    str[i] = '\0';
    return i;
}

/// \brief Get pow of a number
/// \params double n - nubmer
///         int32_t p - power
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
/// \return None
void m_ftoa(double n, char* res, int32_t afterpoint)
{
    // Extract integer part
    int32_t ipart = (int32_t)n;

    // Extract floating part
    double fpart = n - (double)ipart;

    // convert integer part
    int32_t i = 0;
    if ((ipart == 0) && (n < 0))
    {
        res[0] = '-';
        res[1] = '0';
        i = 2;
    }
    else
    {
        i = m_itoa(ipart, res, 0, 1);
    }

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * m_pow(10, afterpoint);

        m_itoa((int32_t)fpart, res + i + 1, afterpoint, 0);
    }
}

/// \brief Get length of string
/// \param char* str - string
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
