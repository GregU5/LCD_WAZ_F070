/*
 * uart_test.h
 *
 *  Created on: May 27, 2018
 *      Author: greg
 */

#ifndef UART_TEST_H_
#define UART_TEST_H_

#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>

#define APB1_CLK 48000000
#define RX_BUFF_SIZE	20
#define TX_BUFF_SIZE	20


void uart2_init(uint32_t baud);
//void UART_send_byte(uint16_t data);
//void UART_send_char(char * char_data);
//void UART_send_int(uint16_t int_data);
//void UART_send_string(char *s);

int32_t uart2_put_char(char data);
void uart2_put_string(char *s);


#endif /* UART_TEST_H_ */
