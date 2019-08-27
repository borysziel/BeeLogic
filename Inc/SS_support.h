/*
 * Support.h
 *
 *  Created on: 08.02.2018
 *      Author: Tomasz
 */

#ifndef SS_SUPPORT_H_
#define SS_SUPPORT_H_

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "SS_it.h"
#include "adc.h"

#define BUZZER_TIM htim14
#define BUZZER_TIM_REGISTER TIM14

void SS_buzzer_init(void);
void SS_buzzer_start_count(uint16_t hertz_100, uint8_t beeps);
void SS_buzzer_start(uint16_t hertz_100);
void SS_buzzer_start_gap_count(uint16_t hertz_100, uint8_t beeps, uint8_t gap);
void SS_buzzer_start_gap(uint16_t hertz_100, uint8_t gap);
void SS_battery_compute_voltage(void);
uint16_t SS_battery_gett_voltage(void);


#endif /* SS_SUPPORT_H_ */
