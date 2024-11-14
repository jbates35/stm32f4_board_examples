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
#include <stdio.h>

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

/****** CUSTOM TIMER.h CODE STARTS HERE *******
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 **/

/**
  * Timer configuration struct
  * @timer_index: Timer 1, 2, 3, ..., 14
  * @channel: Timer channel
  * @timer_mode: Capture or compare mode, or PWM
  * @interrupt_en: Whether the interrupt is enabled or not
  * @base_clock_freq_hz: Base clock frequency of the timer (deterministic)
  * @timer_freq_hz: Desired frequency of the timer (user required)
  * @prescaler: Pre-scalar to run the timer through
*/

typedef struct {
  uint8_t channel;
  uint8_t timer_mode;
  uint8_t interrupt_en;
  uint8_t output_mode;
  uint32_t base_clock_freq_hz;
  uint32_t timer_freq_hz;
  uint16_t prescaler;
} TimerConfig_t;

typedef struct {
  TIM_TypeDef *p_base_addr;
  TimerConfig_t cfg;
} TimerHandle_t;

enum { TIMER_MODE_COMPARE = 0, TIMER_MODE_CAPTURE = 1, TIMER_MODE_PWM = 2 };
enum { TIMER_INTERRUPT_DISABLED = 0, TIMER_INTERRUPT_ENABLED = 1 };

int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state);

int timer_init(const TimerHandle_t *timer_handle);
void timer_irq_config(uint8_t irq_number, uint8_t en_state);

/****** END OF CUSTOM TIMER.h CODE STARTS HERE *******
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 **/

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
  NVIC->ISER[TIM2_IRQn / 32] |= (1 << (TIM2_IRQn % 32));

  // Timer 2 //

  //Enable counter
  // 1. Select the counter clock (internal, external, prescaler).
  RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos);

  // Set timer 2 as upcounter
  TIM2->CR1 |= (1 << TIM_CR1_DIR_Pos);

  // 2. Write the desired data in the TIMx_ARR and TIMx_CCRx registers.
  TIM2->ARR = 1600;                       // Test these by toggling commenting on and off
  TIM2->CCR1 = (0 << TIM_CCR1_CCR1_Pos);  // Test these by toggling commenting on and off
                                          //
  // Set the prescaler value
  TIM2->PSC = 10000;

  // 3. Set the CCxIE and/or CCxDE bits if an interrupt and/or a DMA request is to be generated.
  TIM2->DIER |= (1 << TIM_DIER_CC1IE_Pos);

  // 4. Select the output mode. For example, one must write OCxM=011, OCxPE=0, CCxP=0 and CCxE=1 to toggle OCx output pin when CNT matches CCRx, CCRx preload is not used, OCx is enabled and active high.
  TIM2->CCER |= (1 << TIM_CCER_CC1E_Pos);
  TIM2->CCER &= ~(1 << TIM_CCER_CC1P_Pos);
  TIM2->CCMR1 = (0b011 << TIM_CCMR1_OC1M_Pos);

  // 5. Enable the counter by setting the CEN bit in the TIMx_CR1 register.CR1_CEN;
  TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos);

  // NOTE: The timer on the STM32 Nucleo F446 board is set to 16MHz
}

void TIM2_IRQHandler(void) {
  if (TIM2->SR & (1 << TIM_SR_CC1IF_Pos)) {
    TIM2->SR &= ~(1 << TIM_SR_CC1IF_Pos);

    // Put any logic here for the interrupt
    GPIO_toggle_output_pin(LED_GREEN_PORT, LED_GREEN_PIN);
  }
}

void EXTI15_10_IRQHandler(void) {
  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    GPIO_toggle_output_pin(LED_GREEN_PORT, LED_GREEN_PIN);
  }
}

/****** CUSTOM TIMER.c CODE STARTS HERE *******
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 **/

int get_timer_ticks(uint32_t base_clock_freq, uint16_t prescaler, uint32_t timer_freq);

#define TIMERS {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14}
#define TIMERS_RCC_OFFSETS {0x44, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x44, 0x44, 0x44, 0x44, 0x40, 0x40, 0x40}
#define TIMERS_RCC_POS                                                                                    \
  {                                                                                                       \
      RCC_APB2ENR_TIM1EN_Pos,  RCC_APB1ENR_TIM2EN_Pos,  RCC_APB1ENR_TIM3EN_Pos,  RCC_APB1ENR_TIM4EN_Pos,  \
      RCC_APB1ENR_TIM5EN_Pos,  RCC_APB1ENR_TIM6EN_Pos,  RCC_APB1ENR_TIM7EN_Pos,  RCC_APB2ENR_TIM8EN_Pos,  \
      RCC_APB2ENR_TIM9EN_Pos,  RCC_APB2ENR_TIM10EN_Pos, RCC_APB2ENR_TIM11EN_Pos, RCC_APB1ENR_TIM12EN_Pos, \
      RCC_APB1ENR_TIM13EN_Pos, RCC_APB1ENR_TIM14EN_Pos,                                                   \
  }

#define TIMER_ARR_SIZE(arr) ((int)sizeof(arr) / sizeof(TIM_TypeDef *))
#define TIMER_POS_ARR_SIZE(arr) ((int)sizeof(arr) / sizeof(uint8_t))

int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state) {
  if (base_addr == NULL) return -1;

  // Grab the index of the current timer
  const TIM_TypeDef *timers_arr[] = TIMERS;
  int i = 0;
  for (; i < TIMER_ARR_SIZE(timers_arr); i++) {
    if (timers_arr[i] == base_addr) break;
  }

  // Do nothing if the index is out of range
  if (i >= TIMER_ARR_SIZE(timers_arr)) return -1;

  uint32_t timer_rcc_pos_arr[] = TIMERS_RCC_POS;
  uint32_t timer_rcc_offsets_arr[] = TIMERS_RCC_OFFSETS;

  // Turn on either a register in APB1 or APB2 according to the timer addr
  *(volatile uint32_t *)(RCC_BASE + timer_rcc_offsets_arr[i]) |= (1 << timer_rcc_pos_arr[i]);

  return 0;
}

int timer_init(const TimerHandle_t *timer_handle) {
  if (timer_handle == NULL) return -1;

  TIM_TypeDef *timer = (timer_handle->p_base_addr);
  const TimerConfig_t *cfg = &(timer_handle->cfg);

  // Set the easy ones from the config
  timer->PSC = cfg->prescaler;

  // Set either timer as input or output depending on mode
  if (cfg->timer_mode == TIMER_MODE_COMPARE || cfg->timer_mode == TIMER_MODE_PWM) {
    timer->CR1 |= (1 << TIM_CR1_DIR_Pos);

    // Calculate the necessary clock frequency
  } else if (cfg->timer_mode == TIMER_MODE_CAPTURE) {
  }

  // Set interrupt if required
  if (cfg->interrupt_en == TIMER_INTERRUPT_ENABLED) {
  }

  return 0;
}

int get_timer_ticks(uint32_t base_clock_freq, uint16_t prescaler, uint32_t timer_freq) {
  unsigned int true_base_clock = base_clock_freq / prescaler;
}

void timer_irq_interrupt_config(uint8_t irq_number, uint8_t en_state) {
  // Enables or disables NVIC
  // Programs ISER if enable, programs ICER if disable
  if (en_state)
    NVIC->ISER[irq_number / 32] |= (1 << (irq_number % 32));
  else
    NVIC->ICER[irq_number / 32] |= (1 << (irq_number % 32));
}

void timer_irq_priority_config(uint8_t irq_number, uint8_t irq_priority) {
  uint8_t qshift = (irq_number * 8) % 32;  // Will result in bits 0 - 240*8
  uint8_t qindex = (irq_number * 8) / 32;

  NVIC->IP[qindex] &= ~(0xFF << qshift);
  NVIC->IP[qindex] |= (irq_priority << qshift << 4) & (0xF0 << qshift);
}

int timer_irq_handling(TIM_TypeDef *timer, uint8_t channel) {
  const uint8_t status_regs[] = {TIM_SR_CC1IF_Pos, TIM_SR_CC2IF_Pos, TIM_SR_CC3IF_Pos, TIM_SR_CC4IF_Pos};

  if (timer->SR & (1 << status_regs[channel - 1])) {
    TIM2->SR &= ~(1 << status_regs[channel - 1]);
    return 1;
  }

  return 0;
}
