/*
 * lcd_driver.c
 *
 *  Created on: Jul 3, 2018
 *      Author: greg
 */


#include "lcd_driver.h"
#include "../system/global_timers.h"

volatile uint32_t lcd_tick_ms;

uint32_t lcd_write(struct lcd_port_init *lcd, uint8_t data);
uint32_t lcd_write_data(struct lcd_port_init *lcd, uint8_t data);
uint32_t lcd_write_cmd(struct lcd_port_init *lcd, uint8_t data);
void lcd_send_half(struct lcd_port_init *lcd, uint8_t bits);

void
lcd_send_half(struct lcd_port_init *lcd, uint8_t bits)
{
  if(bits & 0x01) {
    lcd->DB4_PORT->ODR |= (1U << lcd->LCD_PIN.DB4);
  }else {
    lcd->DB4_PORT->ODR &= ~(1U << lcd->LCD_PIN.DB4);
  }

  if(bits & 0x02) {
    lcd->DB5_PORT->ODR |= (1U << lcd->LCD_PIN.DB5);
  }else {
    lcd->DB5_PORT->ODR &= ~(1U << lcd->LCD_PIN.DB5);
  }

  if(bits & 0x04) {
    lcd->DB6_PORT->ODR |= (1U << lcd->LCD_PIN.DB6);
  }else {
    lcd->DB6_PORT->ODR &= ~(1U << lcd->LCD_PIN.DB6);
  }

  if(bits & 0x08) {
    lcd->DB7_PORT->ODR |= (1U << lcd->LCD_PIN.DB7);
  }else {
    lcd->DB7_PORT->ODR &= ~(1U << lcd->LCD_PIN.DB7);
  }
}

uint32_t
lcd_write(struct lcd_port_init *lcd, uint8_t data)
{
  uint32_t retval = 0;
  static uint32_t lcd_driver_state = LCD_STATE_WRITE;
  static uint32_t lcd_tick_ms = 0;

  switch (lcd_driver_state) {
    case LCD_STATE_WRITE: {
      //set E
      lcd->E_PORT->ODR |= (1U << lcd->LCD_PIN.E);
      lcd_send_half(lcd ,data >> 4);
      //disable E
      lcd->E_PORT->ODR &= ~(1U << lcd->LCD_PIN.E);
      //set E
      lcd->E_PORT->ODR |= (1U << lcd->LCD_PIN.E);
      lcd_send_half(lcd, data);
      lcd->E_PORT->ODR &= ~(1U << lcd->LCD_PIN.E);
      lcd_tick_ms = get_system_tick_ms() + 2;
      lcd_driver_state = LCD_STATE_WAIT;
      retval = lcd_driver_state;
    }
    break;

    case LCD_STATE_WAIT: {
      uint32_t tick = get_system_tick_ms();
      if (tick > lcd_tick_ms){
	lcd_driver_state = LCD_STATE_END;
      }
      retval = LCD_STATE_WAIT;
    }
    break;

    case LCD_STATE_END:	{
      lcd_driver_state = LCD_STATE_WRITE;
      retval = LCD_STATE_END;
    }
    break;
  }

  return retval;
}

uint32_t
lcd_write_data(struct lcd_port_init *lcd, uint8_t data)
{
  uint32_t retval = 0;
  //set RS
  lcd->RS_PORT->ODR |= lcd->LCD_PIN.RS;
  while((lcd_write(lcd, data)) != 2);
  return retval;
}

uint32_t
lcd_write_cmd(struct lcd_port_init *lcd, uint8_t cmd)
{
  uint32_t retval = 0;
  lcd->RS_PORT->ODR &= ~lcd->LCD_PIN.RS;
  lcd_write(lcd, cmd);
  while((lcd_write(lcd, cmd)) != 2);
  return retval;
}

int32_t
lcd_init(struct lcd_port_init *lcd)
{
  uint32_t bit_pos = 0;
  uint32_t set_moder = 0;

  bit_pos = lcd->LCD_PIN.RS;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->RS_PORT->MODER	|= set_moder;

  bit_pos = lcd->LCD_PIN.E;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->E_PORT->MODER	|= set_moder;

  bit_pos = lcd->LCD_PIN.DB4;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->DB4_PORT->MODER 	|= set_moder;

  bit_pos = lcd->LCD_PIN.DB5;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->DB5_PORT->MODER 	|= set_moder;

  bit_pos = lcd->LCD_PIN.DB6;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->DB6_PORT->MODER 	|= set_moder;

  bit_pos = lcd->LCD_PIN.DB7;
  bit_pos = bit_pos*2;
  set_moder = (1U << bit_pos);
  lcd->DB7_PORT->MODER 	|= set_moder;
  system_delay_ms(15);

  lcd->RS_PORT->ODR &= ~(1U << lcd->LCD_PIN.RS);
  lcd->E_PORT->ODR &= ~(1U << lcd->LCD_PIN.E);

  for(uint32_t i = 0; i < 3; i++) {
    lcd->E_PORT->ODR |= (1U << lcd->LCD_PIN.E);
    lcd_send_half(lcd, 0x03);
    lcd->E_PORT->ODR &= ~(1U << lcd->LCD_PIN.E);
    system_delay_ms(5);
  }

  lcd->E_PORT->ODR |= (1U << lcd->LCD_PIN.E);
  lcd_send_half(lcd, 0x02);
  lcd->E_PORT->ODR &= ~(1U << lcd->LCD_PIN.E);
  system_delay_ms(1);

  lcd_write_cmd(lcd, (KS0070B_FUNCTION_SET | KS0070B_FONT5x10 | KS0070B_TWO_LINE | KS0070B_4_BIT) ); //interface 4bits, 2-lines, sign 5x7
  lcd_write_cmd(lcd, (KS0070B_DISPLAY_ONOFF | KS0070B_DISPLAY_OFF));
  lcd_write_cmd(lcd, KS0070B_CLEAR); // clear DDR
  system_delay_ms(1);

  lcd_write_cmd(lcd, (KS0070B_ENTRY_MODE | KS0070B_EM_SHIFT_CURSOR | KS0070B_EM_INCREMENT) );// inkrementaja adresu i przesuwanie kursora
  lcd_write_cmd(lcd, (KS0070B_DISPLAY_ONOFF | KS0070B_DISPLAY_ON | KS0070B_CURSOR_ON | KS0070B_CURSOR_BLINK) ); // w³¹cz LCD, bez kursora i mrugania
  system_delay_ms(1);

  return 0;
}

void
lcd_goto(struct lcd_port_init *lcd, uint32_t lcd_y, uint32_t lcd_x)
{
  switch(lcd_y) {
    case 0:
      lcd_write_cmd(lcd, (KS0070B_DDRAM_SET | lcd_x));
    break;

    case 1:
      lcd_write_cmd(lcd, (KS0070B_DDRAM_SET | (0x40+lcd_x)));
    break;

    case 2:
      lcd_write_cmd(lcd, (KS0070B_DDRAM_SET | (0x14+lcd_x)));
    break;

    case 3:
      lcd_write_cmd(lcd, (KS0070B_DDRAM_SET | (0x54+lcd_x)));
    break;
  }
}

void
lcd_show_text(struct lcd_port_init *lcd,char *text)
{
  while(*text) {
    lcd_write_data(lcd, *text++);
  }
}

