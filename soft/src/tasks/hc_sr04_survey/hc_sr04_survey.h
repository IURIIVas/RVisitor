/// \file hc_sr04_survey.h
/// \brief HC-SR04 ultrasonic distance sensor survey header file
/// \author 1jura.vas@gmail.com
///
/// \details
///
///
//--------------------------------------------------- Include guards ---------------------------------------------------

#ifndef HC_SR04_SURVEY_H_
#define HC_SR04_SURVEY_H_

//----------------------------------------------------- Includes -------------------------------------------------------

#include "global_inc.h"
#include "queue.h"

//------------------------------------------------------ Macros --------------------------------------------------------

#define HC_SR04_SENSORS_NUM					  (4)

#define HC_SR04_GPIO_PORT					  (GPIOC)
#define HC_SR04_GPIO_TRIG_PIN				  (GPIO_Pin_13)
#define HC_SR04_GPIO_ECHO_PIN				  (GPIO_Pin_8)
#define HC_SR04_MUX_S0_GPIO_PIN				  (GPIO_Pin_4)
#define HC_SR04_MUX_S1_GPIO_PIN				  (GPIO_Pin_5)
#define HC_SR04_RCC_GPIO					  (RCC_APB2Periph_GPIOC)

#define HC_SR04_TRIG_TIMER					  (TIM6)
#define HC_SR04_TRIG_TIMER_RCC			      (RCC_APB1Periph_TIM6)

#define HC_SR04_ECHO_TIMER					  (TIM8)
#define HC_SR04_ECHO_TIMER_RCC			      (RCC_APB2Periph_TIM8)

#define DISTANCE_THRESHOLD_CM                 (5)

#define HC_SR04_SURVEY_TASK_PRIORITY     	  (5)
#define HC_SR04_SURVEY_TASK_STK_SIZE      	  (256)
#define HC_SR04_DELAY_MS					  (50)

//----------------------------------------------------- Typedefs -------------------------------------------------------


//---------------------------------------------------- Variables -------------------------------------------------------

extern TaskHandle_t hc_sr04_survey_task_handler;
extern uint16_t hc_sr04_sensors_distance[];
extern QueueHandle_t queue_hc_sr04;

//------------------------------------------------ Function prototypes -------------------------------------------------

void hc_sr04_task_init(void);

#endif /* HC_SR04_SURVEY_H_ */
