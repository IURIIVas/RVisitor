/// \file global_inc.h
/// \brief main includes
/// \author 1jura.vas@gmail.com
///
/// \details Include this file to almost every file.
/// 		 Contains required header files.
///


#ifndef GLOBAL_INC_H_
#define GLOBAL_INC_H_

#include "ch32v30x.h"
#include "FreeRTOS.h"
#include "task.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define abs(a)    (((a) > 0) ? (a) : -(a))

#define PI                  (3.14)

#endif
