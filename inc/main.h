/*
 * main.h
 *
 *  Created on: May 24, 2018
 *      Author: greg
 */

#ifndef MAIN_H_
#define MAIN_H_



#include "../system/rcc_clocks.h"
#include "../UART/uart_test.h"
#include "../LCD/lcd_driver.h"


enum {
  CLK_INIT = 0,
  APP_INIT,
  APP_HANDLER,
  APP_ERROR
};


#endif /* MAIN_H_ */
