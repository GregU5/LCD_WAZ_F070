/*
 * rcc_timers.h
 *
 *  Created on: May 23, 2018
 *      Author: greg
 */

#ifndef RCC_CLOCKS_H_
#define RCC_CLOCKS_H_


#include "stm32f0xx.h"

#define HSE_STARTUP_TIMEOUT  100U

void setup_sys_flash(void);
int32_t setup_sys_clocking(void);


#endif /* RCC_CLOCKS_H_ */
