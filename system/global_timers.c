/*
 * global_timer.c
 *
 *  Created on: Jun 21, 2018
 *      Author: greg
 */

#include "global_timers.h"

static volatile uint32_t system_tick_ms = 0L;

void
init_timers(void)
{
  SysTick_Config( SYSCLOCK / 1000);
}

void
system_delay_ms (uint16_t delay_ms)
{
  uint32_t old_tick = get_system_tick_ms();
  while( delay_ms > get_system_tick_ms() - old_tick);
}


uint32_t
get_system_tick_ms(void)
{
  uint32_t ticks = system_tick_ms;
  return ticks;
}


void
SYSTICK_HANDLER(void)
{
  system_tick_ms++;
}
