/*
 * keyboard_4x4.h
 *
 *  Created on: Jun 1, 2018
 *      Author: greg
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

/*
 * keyboard_4x4.h
 *
 *  Created on: May 24, 2018
 *      Author: greg
 */

/*
 * C - COLUMNS, 	R - ROWS
 * PB2 - C3(K3)		PB6 - R0(ROW1)
 * PB3 - C2(K2)		PB7 - R1(ROW2)
 * PB4 - C1(K1)		PB8 - R2(ROW3)
 * PB5 - C0(K0)		PB9 - R3(ROW4)
 *
 *
 * 		R3	R2	R1	R0
 * 	C3	7(4)	8(3)	9(2)	->BTN_A(1)
 *
 * 	C2	4(8)	5(7)	6(6)	->BTN_B(5)
 *
 * 	C1	1(12)	2(11)	3(10)	->BTN_C(9)
 *
 * 	C0	0(16)	S(15)	E(14)	->BTN_D(13)
 *
 *
 *
 * 	---------------------------------------
 *
 * 	PC13 - USER BTN
 *
 *
 * 	KP_FIRST_ROW 4		//exm pc4-pc7 rows ( PB6-PB9 )
 * 	KP_FIRST_COL 0		//exm pc0-pc3 cols ( PB2-PB5 )
 *
 * 	KP_ROW_MASK 0b11110000
 * 	KP_COL_MASK 0B00001111
 *
 */


#include "stm32f0xx.h"

#define KP_IRQHANDLER		TIM7_IRQHandler
#define KP_IRQ			TIM7_IRQn

/*
 * here we define keypad size and assigned first bit of rows or colums bit
 * important note: rows and columns might be assigned to diffrent ports, but
 * each subsequent row must be sequentially assigned to the pin of the port
 * example:
 * ROWS: 		COLUMNS:
 * 	PB0 - ROW1		PC7 -  COL1
 * 	PB1 - ROW2		PC8 -  COL2
 * 	PB2 - ROW3		PC9 -  COL3
 * 	PB3 - ROW4		PC10 - COL4
 *
 * this configuration is NOT allowed:
 * ROWS:		COLUMNS:
 * 	PB0 - ROW1		PA0 = COL1
 * 	PA5 - ROW2		PA8 = COL2
 * 	PD1 - ROW3		PA4 = COL3
 * 	PE0 - ROW4		PC2 = COL4
 *
 * because lib do not support this kind of configuration, yet :-)
 *
 */

#define KP_ROW_PORT		GPIOB
#define KP_COL_PORT		GPIOB

#define KP_ROW_PULLUP		1
#define KP_COL_PULLUP		1

#define KP_FIRST_ROW_BIT 	6
#define KP_FIRST_COL_BIT 	2
#define KP_NUM_COLS 		4
#define KP_NUM_ROWS 		4

#define KP_LED_PORT		GPIOC
#define KP_NUM_LEDS		1
#define KP_FIRST_LED_BIT	9

/*
 * keypad states
 */
enum {
  KP_SCAN,		//scan for pushed button
  KP_ISPRESS,		//check if button was pushed
  KP_PRESSED		//return pushed button
};


struct kp_timer_init {
  uint32_t apb1_freq_mhz;
  uint32_t scan_delay;
};

struct kp_port_init {
  GPIO_TypeDef * port_row;
  GPIO_TypeDef * port_col;
  uint32_t col_mask;
  uint32_t row_mask;
};

struct kp_init {
  uint8_t pullup_col;
  uint8_t pullup_row;

  struct kp_port_init kp_port;
  struct kp_timer_init kp_tim;
};

/*function to init keypad*/
int32_t kp_keypad_init(void);
/*
 * function returned scanned button from keypad
 * if return 0 - button wasnt pushed
 * return value 1-16 value of pushed buttons
 */
uint8_t kp_get_raw_value(void);
/*
 * function assigned characters to the keypad
 * return assigned char from pushed button
 */
uint8_t kp_get_btn(void);
int32_t kp_gpio_toggle_bit(GPIO_TypeDef *port, uint32_t bit_num);


#endif /* KEYPAD_H_ */
