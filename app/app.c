/*
 * app.c
 *
 *  Created on: Jul 3, 2018
 *      Author: greg
 */

#include "app.h"
#include "../LCD/lcd_driver.h"


static struct lcd_port_init LCD_KS4X20;

void
app_init(void)
{

  GPIOA->MODER |= GPIO_MODER_MODER5_0;
  GPIOA->ODR |= GPIO_ODR_5;

  init_timers();

  system_delay_ms(50);

  LCD_KS4X20.E_PORT = GPIOC;
  LCD_KS4X20.LCD_PIN.E = 11;
  LCD_KS4X20.RS_PORT = GPIOC;
  LCD_KS4X20.LCD_PIN.RS = 10;

  LCD_KS4X20.DB7_PORT = GPIOA;
  LCD_KS4X20.DB6_PORT = GPIOA;
  LCD_KS4X20.DB5_PORT = GPIOA;
  LCD_KS4X20.DB4_PORT = GPIOA;

  LCD_KS4X20.LCD_PIN.DB7 = 12;
  LCD_KS4X20.LCD_PIN.DB6 = 11;
  LCD_KS4X20.LCD_PIN.DB5 = 10;
  LCD_KS4X20.LCD_PIN.DB4 = 9;

  lcd_init(&LCD_KS4X20);

  lcd_goto(&LCD_KS4X20, 1,0);
  lcd_show_text(&LCD_KS4X20, "Hello");

}
