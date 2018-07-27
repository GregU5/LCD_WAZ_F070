/*
 * global_timer.h
 *
 *  Created on: Jun 21, 2018
 *      Author: greg
 */

#ifndef GLOBAL_TIMERS_H_
#define GLOBAL_TIMERS_H_

#include "stm32f0xx.h"


#define SYSCLOCK 48000000 //48MHz
#define SYSTICK_HANDLER			SysTick_Handler

void system_delay_ms(uint16_t delay_ms);
void init_timers(void);

uint32_t get_system_tick_ms(void);

#endif /* GLOBAL_TIMERS_H_ */
