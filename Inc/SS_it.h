/*
 * SS_it.h
 *
 *  Created on: 08.02.2018
 *      Author: Tomasz
 */

#ifndef SS_IT_H_
#define SS_IT_H_


#include "stm32f4xx_hal.h"
#include "tim.h"

uint8_t beeps;
uint8_t gap;

void set_beeps(uint8_t beep);
void set_gap(uint8_t gap);

#endif /* SS_IT_H_ */
