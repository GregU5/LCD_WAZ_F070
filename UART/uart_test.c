/*
 * uart_test.c
 *
 *  Created on: May 27, 2018
 *      Author: greg
 */


#include "../UART/uart_test.h"

volatile char uart_rx_buff[RX_BUFF_SIZE];
volatile char uart_tx_buff[TX_BUFF_SIZE];

struct buff {
  volatile char * const buffer;
  uint32_t head;
  uint32_t tail;
};

volatile struct buff rx_rbuff = {
  uart_rx_buff,
  0,
  0
};

volatile struct buff tx_rbuff = {
    uart_tx_buff,
    0,
    0
};

void
uart2_init(uint32_t baud)
{
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // podpiecie zegara dla uarta
  GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1); //alternate mode
  GPIOA->AFR[0] |= (uint32_t) (1<<8) | (1<<12); //alternate(AF7) for PA2 and PA3
  USART2->BRR = (APB1_CLK/baud);
  USART2->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
  NVIC_EnableIRQ(USART2_IRQn);
}

//void
//UART_send_char(char * char_data)
//{
//  UART_send_byte(*char_data);
//}
//
//void
//UART_send_string(char *s)
//{
//  while( *(s) != 0 ){
//    UART_send_byte(*s);
//    s++;
//  }
//}
//
//void
//UART_send_byte(uint16_t data)
//{
//  while(! (USART2->ISR & USART_ISR_TXE));
////  USART2->TDR = (data & 0xFF);
//  USART2->TDR = (data & 0xFFFF);
//}
//
//void
//UART_send_int(uint16_t int_data)
//{
//  char buffer[30];
//  itoa(int_data, buffer, 10);
//  UART_send_string(buffer);
//}

/*
 *
 * TODO: USE RING BUFFER TO CATCH DATA FROM SERIAL PORT
 * BUFFED LAYER OF THE SCREEN ?
 *
 */

/*
 *  buffed functions
 */

int32_t
uart2_put_char(char data)
{
  uint8_t head_temp = tx_rbuff.head + 1;

  if (head_temp == TX_BUFF_SIZE) {
    head_temp = 0;
  }
  while (head_temp == tx_rbuff.tail) {

  }

  tx_rbuff.buffer[head_temp] = data;
  tx_rbuff.head = head_temp;

  /* TX IT enable */
  USART2->CR1 |= USART_CR1_TXEIE;

  return 0;
}

int32_t
uart2_get_char(char *data)
{
  uint32_t head = rx_rbuff.head;
  uint32_t tail = rx_rbuff.tail;

  if (head == tail) {
    return -1;
  }
  tail++;

  if (tail == RX_BUFF_SIZE) {
    tail = 0;
  }

  *data = rx_rbuff.buffer[tail];

  rx_rbuff.tail = tail;
  rx_rbuff.head = head;

  return 0;
}

void
uart2_put_string( char *s)
{
  while(*s) {
    uart2_put_char(*s++);
  }
}

/* INTERUPTS */
void
USART2_IRQHandler(void)
{
  /* odbior */
  if ( USART2->ISR & USART_ISR_RXNE) {
    uint32_t head_temp = rx_rbuff.head + 1;
    if (head_temp == RX_BUFF_SIZE) {
      head_temp = 0;
    }

    if(head_temp == rx_rbuff.tail) {
      //buffor pelny

      USART2->ISR &= ~USART_ISR_RXNE;
    }else {
	rx_rbuff.buffer[head_temp] = USART2->RDR;


    }
    rx_rbuff.head = head_temp;
  }


  /* nadawanie */
  if (USART2->ISR & USART_ISR_TXE) {
      uint32_t tail_temp = tx_rbuff.tail;
      //bufor pusty, wylacz przerwanie
      if (tx_rbuff.head == tail_temp ) {
	USART2->CR1 &= ~USART_CR1_TXEIE;
      }else {
	tail_temp++;
	if(tail_temp == TX_BUFF_SIZE) {
	  tail_temp = 0;
	}
	USART2->TDR = tx_rbuff.buffer[tail_temp];

      }
      tx_rbuff.tail = tail_temp;
  }
}
