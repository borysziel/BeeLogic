/*
 * Support.c
 *
 *  Created on: 08.02.2018
 *      Author: Tomasz
 */

#include "SS_support.h"

/* configure timer prescaler for buzzer to make without ARR register 1000kHz period */
static uint16_t battery_voltage;

void SS_buzzer_start(uint16_t hertz_100)
{
	BUZZER_TIM_REGISTER->ARR = 1000 *100/hertz_100 - 1;
	set_gap(2);
	HAL_TIM_Base_Start_IT(&BUZZER_TIM);
}
void SS_buzzer_stop(void)
{
	HAL_TIM_Base_Stop_IT(&BUZZER_TIM);
}
void SS_buzzer_start_count(uint16_t hertz_100, uint8_t beeps)
{
	BUZZER_TIM_REGISTER->ARR = 1000 *100/hertz_100 - 1;
	set_beeps(beeps);
	set_gap(2);
	HAL_TIM_Base_Start_IT(&BUZZER_TIM);
}
void SS_buzzer_start_gap_count(uint16_t hertz_100, uint8_t beeps, uint8_t gap)
{
	BUZZER_TIM_REGISTER->ARR = 1000 *100/hertz_100 - 1;
	set_beeps(beeps);
	set_gap(gap*2);
	HAL_TIM_Base_Start_IT(&BUZZER_TIM);
}
void SS_buzzer_start_gap(uint16_t hertz_100, uint8_t gap)
{
	BUZZER_TIM_REGISTER->ARR = 1000 *100/hertz_100 - 1;
	set_beeps(255);
	set_gap(gap*2);
	HAL_TIM_Base_Start_IT(&BUZZER_TIM);
}


