/*
 * keyboard_4x4.c
 *
 *  Created on: May 24, 2018
 *      Author: greg
 */
#include "../keypad/keypad.h"


int32_t kp_scanner_tim_init(uint32_t f_MHz, uint32_t scan_delay_ms);
int32_t kp_set_gpio_mask(struct kp_port_init * kp_port);

int32_t kp_gpio_set_pullups(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit);
int32_t kp_gpio_set_as_outputs(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit);
int32_t kp_gpio_set_bits(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit);


void kp_scanner_start(void);
void kp_scanner_stop(void);

uint32_t kp_scan_keypad(void);


struct kp_port_init keypad_port;
struct kp_scanner {
  volatile uint32_t scan_time_ms;
} keypad_scanner;

uint8_t active_buttons[] = {
      '7', '8', '9', 'A',
      '4', '5', '6', 'B',
      '1', '2', '3', 'C',
      '0', 'S', 'E', 'D'
 };


int32_t
kp_scanner_tim_init(uint32_t f_MHz, uint32_t scan_delay_ms)
{
  int32_t retval = 0;
  /*timer init for keyboard handling*/
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  TIM7->CR1 |= TIM_CR1_URS;
  /* IRQ ENABLE */
  TIM7->DIER |= TIM_DIER_UIE;
  NVIC_EnableIRQ(KP_IRQ);
  /* freq = 48MHz, PSC = 48.000, TIM speed 1kHz */
  TIM7->PSC = (f_MHz * 10000) - 1;
  /* miliseconds */
  TIM7->ARR = scan_delay_ms - 1; // IRQ EACH EVERY 10ms

  return retval;
}

int32_t
kp_keypad_init(void)
{
  int32_t retval = 0;
  struct kp_init keypad;

  keypad.kp_port.port_col = KP_COL_PORT;
  keypad.kp_port.port_row = KP_ROW_PORT;
  keypad.pullup_col = KP_COL_PULLUP;
  keypad.pullup_row = KP_ROW_PULLUP;

  kp_scanner_tim_init(48, 20);
  /* port of COLS configured as OUTPUTS, port of ROWS configured as INPUTS */
  kp_gpio_set_as_outputs(keypad.kp_port.port_col, KP_NUM_COLS, KP_FIRST_COL_BIT);
  /* set PULL UP for COLUMNS */
  if(keypad.pullup_col == 1) {
    kp_gpio_set_pullups(keypad.kp_port.port_col, KP_NUM_COLS, KP_FIRST_COL_BIT);
  }
  /* set PULL UP for ROWS */
  if(keypad.pullup_row == 1) {
    kp_gpio_set_pullups(keypad.kp_port.port_row, KP_NUM_ROWS, KP_FIRST_ROW_BIT);
  }
  /* output IDLE state for COLUMNS and ROWS */
  kp_gpio_set_bits(keypad.kp_port.port_col, KP_NUM_COLS, KP_FIRST_COL_BIT);
  kp_gpio_set_bits(keypad.kp_port.port_row, KP_NUM_ROWS, KP_FIRST_ROW_BIT);

  keypad_port.port_col = keypad.kp_port.port_col;
  keypad_port.port_row = keypad.kp_port.port_row;
  kp_set_gpio_mask(&keypad_port);

  /*KEYPAD LED*/
  /*
   * TODO: add function or macro to drive keypad led
   */
  kp_gpio_set_bits(KP_LED_PORT, KP_NUM_LEDS, KP_FIRST_LED_BIT);
  kp_gpio_set_as_outputs(KP_LED_PORT, KP_NUM_LEDS, KP_FIRST_LED_BIT);

  GPIOC->ODR |= GPIO_ODR_9;
  GPIOC->MODER |= GPIO_MODER_MODER9_0;

  return retval;
}

/* return raw not filtered readed value from pushed button */
uint32_t
kp_scan_keypad(void)
{
  volatile uint32_t retval = 0;

  volatile uint32_t row_num = 0;
  volatile uint32_t row_bit = 0;

  volatile uint32_t col_num = 0;
  volatile uint32_t col_bit = 0;

  volatile uint32_t rows = 0;

  col_bit = 1 << KP_FIRST_COL_BIT;		//zapamietaj bit kolumny

  for(col_num = 0; col_num < 4; col_num++) {
    keypad_port.port_col->ODR &= ~col_bit;

    col_bit <<= 1;			//przesuń bit kolumny
    row_bit = 1 << KP_FIRST_ROW_BIT; 	//zapmietaj pierwszy bit WIERSZA
    rows = ~(keypad_port.port_row->IDR & (keypad_port.row_mask)) & keypad_port.row_mask;

    keypad_port.port_col->ODR |= keypad_port.col_mask;

    for(row_num = 0; row_num < KP_NUM_COLS; row_num++) {
      if(row_bit & rows) {
	return retval = (col_num * KP_NUM_ROWS) + row_num +1;
      }
      else {
	row_bit <<= 1;
      }
    }
  }
  return retval;
}

/*return 0 - button not pressed */
uint8_t
kp_get_raw_value(void)
{
  uint8_t retval = 0;
  static uint32_t state = KP_SCAN;
  static uint32_t last_val = 0;
  uint8_t curr_val = 0;

  switch (state) {

    case KP_SCAN: {
      keypad_scanner.scan_time_ms = 0;
      curr_val = kp_scan_keypad();
      if(curr_val != 0) {
	keypad_scanner.scan_time_ms = 3;
	kp_scanner_start();
	last_val = curr_val;
	state = KP_ISPRESS;
      }
    }
    break;

    case KP_ISPRESS: {
      curr_val = kp_scan_keypad();
      if(keypad_scanner.scan_time_ms == 0) {
	if(curr_val == last_val) {
	  kp_scanner_stop();
	  retval = curr_val;
	  state = KP_PRESSED;
	}
	else {

	  state = KP_SCAN;
	}
      }
    }
    break;

    case KP_PRESSED: {
      curr_val = kp_scan_keypad();
      if(last_val == curr_val) {
	retval = 0;
      }
      else {
	  state = KP_SCAN;
      }
    }
  }
  return retval;
}

uint8_t
kp_get_btn(void)
{
  uint8_t key = 0;
  uint8_t raw_num = kp_get_raw_value();

  if(raw_num != 0) {
    switch(raw_num) {
      case 16: key = active_buttons[12]; 	// 0
      break;
      case 15: key = active_buttons[13]; 	// 'S'
      break;
      case 14: key = active_buttons[14];	// 'E'
      break;

      case 12: key = active_buttons[8];		// 1
      break;
      case 11: key = active_buttons[9];		// 2
      break;
      case 10: key = active_buttons[10];	// 3
      break;

      case 8: key = active_buttons[4];		// 4
      break;
      case 7: key = active_buttons[5];		// 5
      break;
      case 6: key = active_buttons[6];		// 6
      break;

      case 4: key = active_buttons[0];		// 7
      break;
      case 3: key = active_buttons[1];		// 8
      break;
      case 2: key = active_buttons[2];		// 9
      break;
    }
  }

  return key;
}

int32_t
kp_gpio_set_bits(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit)
{
  int32_t retval = 0;
  uint32_t set_bits = 0;
  for(uint8_t i = 0; i < no_cols_rows; i++) {
    uint32_t bit_pos = first_bit + i;
    set_bits |= (1<<bit_pos);
  }
  port->ODR |= set_bits;

  return retval;
}

int32_t
kp_gpio_set_as_outputs(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit)
{
  int32_t retval = 0;
  uint32_t set_as_output = 0;

  for(uint8_t i = 0; i < no_cols_rows; i++) {
    uint32_t bit_pos = (first_bit*2 + (i*2));
    set_as_output |= (1U << bit_pos);
  }
  port->MODER |= set_as_output;

  return retval;
}

int32_t
kp_gpio_set_pullups(GPIO_TypeDef *port, uint32_t no_cols_rows, uint32_t first_bit)
{
  int32_t retval = 0;
  uint32_t set_pull_up = 0;

  for(uint8_t i = 0; i < no_cols_rows; i++) {
    uint32_t bit_pos = (first_bit*2 + (2*i));
    set_pull_up |= (1U << bit_pos);
  }
  port->PUPDR |= set_pull_up;

  return retval;
}

int32_t
kp_set_gpio_mask(struct kp_port_init * kp_port)
{
  int32_t retval = 0;

  for(int bit_pos = KP_FIRST_COL_BIT; bit_pos < (KP_FIRST_COL_BIT + KP_NUM_COLS); bit_pos++ ) {
    kp_port->col_mask |= (1U<<bit_pos);
  }
  for(int bit_pos = KP_FIRST_ROW_BIT; bit_pos < (KP_FIRST_ROW_BIT + KP_NUM_ROWS); bit_pos++ ) {
    kp_port->row_mask |= (1U<<bit_pos);
  }

  return retval;
}

int32_t
kp_gpio_toggle_bit(GPIO_TypeDef *port, uint32_t bit_num)
{
  int32_t retval = 0;

  port->ODR ^= (1U<<bit_num);

  return retval;
}

void
kp_scanner_start(void)
{
  TIM7->CR1 |= TIM_CR1_CEN;
}

void
kp_scanner_stop(void)
{
  TIM7->CR1 &= ~TIM_CR1_CEN;
  TIM7->CNT = 0;
}

void
KP_IRQHANDLER(void)
{
  volatile uint32_t * p_cc;
  p_cc = &keypad_scanner.scan_time_ms;
  if(TIM7->SR & TIM_SR_UIF) {
    TIM7->SR = ~TIM_SR_UIF;
    if(*p_cc > 0) {
	*p_cc = *p_cc - 1;
    }
  }
}
