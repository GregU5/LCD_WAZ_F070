/*
 * lcd_driver.h
 *
 *  Created on: Jul 3, 2018
 *      Author: greg
 */


#include "stm32f0xx.h"
#include "stdint.h"

enum lcd_states{
  LCD_STATE_WRITE,
  LCD_STATE_WAIT,
  LCD_STATE_END
};

#define KS0070B_CLEAR					0x01
#define KS0070B_HOME					0x02

#define KS0070B_ENTRY_MODE				0x04
enum {
  KS0070B_EM_SHIFT_CURSOR = 0,
  KS0070B_EM_SHIFT_DISPLAY = 1,
  KS0070B_EM_DECREMENT = 0,
  KS0070B_EM_INCREMENT = 2
};

#define KS0070B_DISPLAY_ONOFF				0x08
enum {
  KS0070B_DISPLAY_OFF = 0,
  KS0070B_DISPLAY_ON = 4,
  KS0070B_CURSOR_OFF = 0,
  KS0070B_CURSOR_ON = 2,
  KS0070B_CURSOR_NOBLINK = 0,
  KS0070B_CURSOR_BLINK = 1
};

#define KS0070B_DISPLAY_CURSOR_SHIFT			0x10
enum {
  KS0070B_SHIFT_CURSOR = 0,
  KS0070B_SHIFT_DISPLAY = 8,
  KS0070B_SHIFT_LEFT = 0,
  KS0070B_SHIFT_RIGHT = 4
};

#define KS0070B_FUNCTION_SET				0x20
enum {
  KS0070B_FONT5x7 = 0,
  KS0070B_FONT5x10 = 4,
  KS0070B_ONE_LINE = 0,
  KS0070B_TWO_LINE = 8,
  KS0070B_4_BIT = 0,
  KS0070B_8_BIT = 16
};

#define KS0070B_CGRAM_SET				0x40
#define KS0070B_DDRAM_SET				0x80

struct lcd_pin {
  uint32_t RS;
  uint32_t E;
  uint32_t DB7;
  uint32_t DB6;
  uint32_t DB5;
  uint32_t DB4;
};

struct lcd_port_init {
  GPIO_TypeDef * RS_PORT;
  GPIO_TypeDef * E_PORT;
  GPIO_TypeDef * DB7_PORT;
  GPIO_TypeDef * DB6_PORT;
  GPIO_TypeDef * DB5_PORT;
  GPIO_TypeDef * DB4_PORT;
  struct lcd_pin LCD_PIN;
};

int32_t lcd_init(struct lcd_port_init *lcd);
void lcd_goto(struct lcd_port_init *lcd, uint32_t lcd_y, uint32_t lcd_x);
void lcd_show_text(struct lcd_port_init *lcd, char *text);
