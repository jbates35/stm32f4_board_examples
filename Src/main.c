/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                              \
  do {                                         \
    for (int def_i = 0; def_i < CNT; def_i++); \
  } while (0)

/******* PINS *********/
#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN 5

#define USER_PBUTTON_PORT GPIOC
#define USER_PBUTTON_PIN 13

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void timer_setup(void);

int main(void) {
  // RCC->CR |= RCC_CR_

  GPIO_peri_clock_control(LED_GREEN_PORT, GPIO_CLOCK_ENABLE);
  GPIO_peri_clock_control(USER_PBUTTON_PORT, GPIO_CLOCK_ENABLE);

  GPIO_Handle_t gpio_handle;
  GPIO_TypeDef **addr = &gpio_handle.p_GPIO_x;
  GPIO_PinConfig_t *cfg = &gpio_handle.GPIO_pin_config;

  *addr = LED_GREEN_PORT;
  cfg->GPIO_pin_number = LED_GREEN_PIN;
  cfg->GPIO_pin_mode = GPIO_MODE_OUT;
  cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_NONE;
  cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);

  *addr = USER_PBUTTON_PORT;
  cfg->GPIO_pin_number = USER_PBUTTON_PIN;
  cfg->GPIO_pin_mode = GPIO_MODE_IT_FT;
  cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  cfg->GPIO_pin_pupd_control = GPIO_PUPDR_PULLDOWN;
  cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);
  GPIO_irq_interrupt_config(EXTI15_10_IRQn, GPIO_INT_ENABLE);
  // GPIO_irq_priority_config(EXTI15_10_IRQn, USER_PBUTTON_PIN);

  timer_setup();

  /* Loop forever */
  for (;;) {
  }
}

void timer_setup(void) {
  // Timer 2 //

  //Enable counter
  // 1. Select the counter clock (internal, external, prescaler).
  RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos);

  // Set timer 2 as upcounter
  TIM2->CR1 |= (1 << TIM_CR1_DIR_Pos);

  // 2. Write the desired data in the TIMx_ARR and TIMx_CCRx registers.
  TIM2->ARR = 10000;                         // Test these by toggling commenting on and off
  TIM2->CCR1 = (3000 << TIM_CCR2_CCR2_Pos);  // Test these by toggling commenting on and off

  // 3. Set the CCxIE and/or CCxDE bits if an interrupt and/or a DMA request is to be generated.
  TIM2->DIER |= (1 << TIM_DIER_CC1IE_Pos);

  // 4. Select the output mode. For example, one must write OCxM=011, OCxPE=0, CCxP=0 and CCxE=1 to toggle OCx output pin when CNT matches CCRx, CCRx preload is not used, OCx is enabled and active high.
  TIM2->CCER |= (1 << TIM_CCER_CC1E_Pos);
  TIM2->CCER &= ~(1 << TIM_CCER_CC1P_Pos);
  TIM1->CCMR1 = (0b011 << TIM_CCMR1_OC1M_Pos);

  // 5. Enable the counter by setting the CEN bit in the TIMx_CR1 register.CR1_CEN;
  TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos);

  // Set the prescaler value
  TIM2->PSC = 10000;
}

void TIM2_IRQHandler(void) {
  if (TIM2->SR & (1 << TIM_SR_CC1IF_Pos)) {
    TIM2->SR &= ~(1 << TIM_SR_CC1IF_Pos);

    // Put any logic here for the interrupt
  }
}

void EXTI15_10_IRQHandler(void) {
  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    GPIO_toggle_output_pin(LED_GREEN_PORT, LED_GREEN_PIN);
  }
}
