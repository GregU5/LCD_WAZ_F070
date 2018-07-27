/*
 * rcc_timers.c
 *
 *  Created on: Apr 5, 2018
 *      Author: greg
 */

#include "../system/rcc_clocks.h"

/* SET 48MHz */

void
setup_sys_flash (void)
{
  FLASH->ACR |= FLASH_ACR_PRFTBE;  // enable prefetch
  FLASH->ACR |= FLASH_ACR_LATENCY; // Set one wait state for 48MHz main clock
}

int32_t
setup_sys_clocking (void)
{
  int32_t retval = 0;
  __IO uint32_t startup_counter = 0;

  RCC->CR |= RCC_CR_HSEON;

  /***** only for F070RB!! *****/
  //RCC->CFGR &= ~(1U<<16U);;//~RCC_CFGR_PLLSRC;
  RCC->CFGR |= (1U<<17U) ;//RCC_CFGR_PLLSRC_HSE_PREDIV;
  RCC->CFGR |= (1U<<16U);
  RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2;

  while (!(RCC->CR & RCC_CR_HSERDY)) { 			// Wait for HSE set-up TODO: Add timeout, return error
    if (startup_counter++ > HSE_STARTUP_TIMEOUT) {
      RCC->CR &= ~RCC_CR_HSEON;
      RCC->CFGR &= ~RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL;
      RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL2; // Set PLL_CLK = 48MHz = 8MHz * 6
      retval = -1;
      break;
    }
  }

  if (retval == 0) {
    RCC->CFGR |= RCC_CFGR_PLLMUL8;			// Set PLL_CLK = 48MHz = 8MHz * 6 --- leave double PLLMUL config for more universal use.
  }
  RCC->CR |= RCC_CR_PLLON; 				// Start PLL clock
  while (!(RCC->CR & RCC_CR_PLLRDY));			// Wait for PLL set-up TODO: Add timeout, return error
  RCC->CFGR |= RCC_CFGR_SW_PLL;				// Choose PLL as a main clock source
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL ));		// Wait for change main clock from HSI to PLL TODO: Add timeout, return error

//  system_set_main_clock_freq(48000000);			// Update information about main clock frequency
//  system_set_apb1_clock_freq(48000000);
//  system_set_apb2_clock_freq(48000000);

  /*
   * Setup RCC regs for enable peripherals.
   */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN
	      | RCC_AHBENR_GPIOCEN  | RCC_AHBENR_GPIODEN; /* Enable GPIOs and DMA1 */
//  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* For exti K inputs*/
  /* Enable the peripheral clock of DBG register (When DEBUG_ENABLE than is uncommented for debug purpose) */
#if defined(DEBUG_ENABLE)
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif
  __DSB();

  return retval;
}
