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

void timer_setup_test(void);

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
  * @gpio_en: Whether the associated GPIO should be set up to follow the timer. Necessary for PWM and Input capture
  * @interrupt_en: Whether the interrupt is enabled or not
  * @ccr_vals: The ccr vals to be set, which dictate when the interrupts happen
  * @timer_mode: Capture or compare mode, or PWM
  * @arr_val: The auto reset reload register for the timer
  * @prescaler: Pre-scalar to run the timer through (actual_freq = base_freq/(psc+1))
  * @channel_count: The number of channels to init (TIM1-5, and 8 have 4 timers, others have 2 timers)
  *
  * NOTE: If using with nucleo f446 board, the base clock will be 16MHz
**/

typedef struct {
  uint8_t gpio_en[4];
  uint8_t interrupt_en[4];
  uint8_t channel_mode[4];
  uint16_t ccr[4];
  uint16_t arr;
  uint16_t prescaler;
  uint8_t channel_count;
  uint8_t dir;
} TimerConfig_t;

/**
  * Timer handler struct
  * @p_base_addr: Address of the timer (Accessed via TIM1, TIM2, ..., TIM14)
  * @cfg: Configuration struct for the timer
  * NOTE: This API only allows for 16-bit values to be compared to in output mode
**/
typedef struct {
  TIM_TypeDef *p_base_addr;
  TimerConfig_t cfg;
} TimerHandle_t;

enum {
  TIMER_CHANNEL_MODE_COMPARE = 0,
  TIMER_CHANNEL_MODE_CAPTURE = 1,
  TIMER_CHANNEL_MODE_PWM_HI = 2,
  TIMER_CHANNEL_MODE_PWM_LO = 3
};
enum { TIMER_INTERRUPT_DISABLE = 0, TIMER_INTERRUPT_ENABLE = 1 };
enum { TIMER_IRQ_DISABLE = 0, TIMER_IRQ_ENABLE = 1 };
enum { TIMER_PERI_CLOCK_DISABLE = 0, TIMER_PERI_CLOCK_ENABLE = 1 };
enum { TIMER_CHANNEL_1 = 0, TIMER_CHANNEL_2 = 1, TIMER_CHANNEL_3 = 2, TIMER_CHANNEL_4 = 3 };
enum { TIMER_GPIO_DISABLE = 0, TIMER_GPIO_ENABLE = 1 };
enum { TIMER_DIR_UP = 0, TIMER_DIR_DOWN = 1 };

int timer_peri_clock_control(const TIM_TypeDef *base_addr, const uint8_t en_state);

int timer_init(const TimerHandle_t *timer_handle);
int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, const uint16_t val);
void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state);
void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority);
int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);

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
  GPIO_TypeDef **gpio_addr = &gpio_handle.p_GPIO_x;
  GPIO_PinConfig_t *gpio_cfg = &gpio_handle.GPIO_pin_config;

  *gpio_addr = LED_GREEN_PORT;
  gpio_cfg->GPIO_pin_number = LED_GREEN_PIN;
  gpio_cfg->GPIO_pin_mode = GPIO_MODE_OUT;
  gpio_cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  gpio_cfg->GPIO_pin_pupd_control = GPIO_PUPDR_NONE;
  gpio_cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  gpio_cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);

  *gpio_addr = USER_PBUTTON_PORT;
  gpio_cfg->GPIO_pin_number = USER_PBUTTON_PIN;
  gpio_cfg->GPIO_pin_mode = GPIO_MODE_IT_FT;
  gpio_cfg->GPIO_pin_speed = GPIO_SPEED_LOW;
  gpio_cfg->GPIO_pin_pupd_control = GPIO_PUPDR_PULLDOWN;
  gpio_cfg->GPIO_pin_out_type = GPIO_OP_TYPE_PUSHPULL;
  gpio_cfg->GPIO_pin_alt_func_mode = 0;
  GPIO_init(&gpio_handle);
  GPIO_irq_interrupt_config(EXTI15_10_IRQn, GPIO_INT_ENABLE);
  // GPIO_irq_priority_config(EXTI15_10_IRQn, USER_PBUTTON_PIN);

  // timer_setup();

  TimerHandle_t timer_handle;
  TIM_TypeDef **timer_addr = &timer_handle.p_base_addr;
  TimerConfig_t *timer_cfg = &timer_handle.cfg;

  // Timer 2 configuration
  timer_peri_clock_control(TIM2, TIMER_PERI_CLOCK_ENABLE);
  *timer_addr = TIM2;

  // overall timer specific
  timer_cfg->prescaler = 253;  // 16M / (PSC + 1) = ~65536
  timer_cfg->arr = 65535;
  timer_cfg->channel_count = 2;

  // channel specific
  timer_cfg->interrupt_en[0] = TIMER_INTERRUPT_ENABLE;
  timer_cfg->interrupt_en[1] = TIMER_INTERRUPT_ENABLE;
  timer_cfg->ccr[0] = 0;
  timer_cfg->ccr[1] = 45000;
  timer_cfg->channel_mode[0] = TIMER_CHANNEL_MODE_COMPARE;
  timer_cfg->channel_mode[1] = TIMER_CHANNEL_MODE_COMPARE;

  // Initialize timer, enable the associated ISR
  timer_init(&timer_handle);
  timer_irq_interrupt_config(TIM2_IRQn, TIMER_IRQ_ENABLE);

  int test_debug_line = 1234;

  /* Loop forever */
  for (;;) {
  }
}

void timer_setup_test(void) {
  NVIC->ISER[TIM2_IRQn / 32] |= (1 << (TIM2_IRQn % 32));

  // Timer 2 //

  //Enable counter
  // 1. Select the counter clock (internal, external, prescaler).
  RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos);

  // Set timer 2 as upcounter
  TIM2->CR1 |= (1 << TIM_CR1_DIR_Pos);

  // 2. Write the desired data in the TIMx_ARR and TIMx_CCRx registers.
  /* TIM2->ARR = 0xFFFF;         // Test these by toggling commenting on and off
  TIM2->CCR1 = 0xFFFF - 800;  // Test these by toggling commenting on and off */
  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0x6000;
  TIM2->ARR = 0xFFFF;
  //
  // Set the prescaler value
  TIM2->PSC = 243;

  // 3. Set the CCxIE and/or CCxDE bits if an interrupt and/or a DMA request is to be generated.
  TIM2->DIER |= (1 << TIM_DIER_CC1IE_Pos);
  TIM2->DIER |= (1 << TIM_DIER_CC2IE_Pos);

  // 4. Select the output mode. For example, one must write OCxM=011, OCxPE=0, CCxP=0 and CCxE=1 to toggle OCx output pin when CNT matches CCRx, CCRx preload is not used, OCx is enabled and active high.
  // TIM2->CCER |= (1 << TIM_CCER_CC1E_Pos);
  // TIM2->CCER &= ~(1 << TIM_CCER_CC1P_Pos);
  TIM2->CCMR1 = (0b001 << TIM_CCMR1_OC1M_Pos);

  // 5. Enable the counter by setting the CEN bit in the TIMx_CR1 register.CR1_CEN;
  TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos);

  // NOTE: The timer on the STM32 Nucleo F446 board is set to 16MHz
}

void TIM2_IRQHandler(void) {
  static int tim1 = 0, tim2 = 0;

  if (timer_irq_handling(TIM2, 1)) {
    GPIO_set_output(LED_GREEN_PORT, LED_GREEN_PIN, 1);
    tim1 = TIM2->CNT;
  }
  if (timer_irq_handling(TIM2, 2)) {
    GPIO_set_output(LED_GREEN_PORT, LED_GREEN_PIN, 0);
    tim2 = TIM2->CNT;
  }
}

void EXTI15_10_IRQHandler(void) {
  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    GPIO_toggle_output(LED_GREEN_PORT, LED_GREEN_PIN);
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

#define TIMERS {TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14}
#define TIMERS_RCC_REGS                                                                                     \
  {&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, \
   &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR}
#define TIMERS_RCC_POS                                                                                    \
  {                                                                                                       \
      RCC_APB2ENR_TIM1EN_Pos,  RCC_APB1ENR_TIM2EN_Pos,  RCC_APB1ENR_TIM3EN_Pos,  RCC_APB1ENR_TIM4EN_Pos,  \
      RCC_APB1ENR_TIM5EN_Pos,  RCC_APB1ENR_TIM6EN_Pos,  RCC_APB1ENR_TIM7EN_Pos,  RCC_APB2ENR_TIM8EN_Pos,  \
      RCC_APB2ENR_TIM9EN_Pos,  RCC_APB2ENR_TIM10EN_Pos, RCC_APB2ENR_TIM11EN_Pos, RCC_APB1ENR_TIM12EN_Pos, \
      RCC_APB1ENR_TIM13EN_Pos, RCC_APB1ENR_TIM14EN_Pos,                                                   \
  }

#define TIMER_ARR_SIZE(arr) ((int)sizeof(arr) / sizeof(TIM_TypeDef *))
#define TIMER_POS_ARR_SIZE(arr) ((int)sizeof(arr) / sizeof(uint8_t))

#define TIMERS_CCR_REGS(timer) {&timer->CCR1, &timer->CCR2, &timer->CCR3, &timer->CCR4}
#define TIMERS_CCMR_REGS(timer) {&timer->CCMR1, &timer->CCMR1, &timer->CCMR2, &timer->CCMR2}

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

  // Turn on either a register in APB1 or APB2 according to the timer addr
  const uint32_t timer_rcc_pos_arr[] = TIMERS_RCC_POS;
  volatile uint32_t *timer_rcc_reg_arr[] = TIMERS_RCC_REGS;
  *timer_rcc_reg_arr[i] |= (1 << timer_rcc_pos_arr[i]);

  return 0;
}

int timer_init(const TimerHandle_t *timer_handle) {
  if (timer_handle == NULL) return -1;

  // Break the addr and config into separate variables for nicer looking code
  TIM_TypeDef *timer = (timer_handle->p_base_addr);
  const TimerConfig_t *cfg = &(timer_handle->cfg);

  // Set the frequency of the timer
  timer->PSC = cfg->prescaler;
  timer->ARR = cfg->arr;

  // Set direction of the timer
  if (cfg->dir == TIMER_DIR_UP)
    timer->CR1 &= ~(1 << TIM_CR1_DIR);
  else if (cfg->dir == TIMER_DIR_DOWN)
    timer->CR1 |= (1 << TIM_CR1_DIR);

  // For easier indexing of addresses
  volatile uint32_t *ccr_reg[] = TIMERS_CCR_REGS(timer);
  volatile uint32_t *ccmr_reg[] = TIMERS_CCMR_REGS(timer);

  // Configure channel specific attributes
  for (int i = 0; i < cfg->channel_count && i < sizeof(ccr_reg) / sizeof(ccr_reg[0]); i++) {
    // Reset the bits for output mode so it can be set by the following
    *ccmr_reg[i] &= ~(0b111 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));

    // Configure the output mode accordingly
    if (cfg->channel_mode[i] == TIMER_CHANNEL_MODE_COMPARE) {
    } else if (cfg->channel_mode[i] == TIMER_CHANNEL_MODE_CAPTURE) {
    } else if (cfg->channel_mode[i] == TIMER_CHANNEL_MODE_PWM_HI) {
      *ccmr_reg[i] |= (0b110 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));
    } else if (cfg->channel_mode[i] == TIMER_CHANNEL_MODE_PWM_LO) {
      *ccmr_reg[i] |= (0b111 << (TIM_CCMR1_OC1M_Pos + (i * 8) % 16));
    }

    // Enable or disable the forwarding of the pin information to the GPIO pin
    if (cfg->gpio_en[i] == TIMER_GPIO_ENABLE)
      timer->CCER |= (1 << (TIM_CCER_CC1E + (4 * i)));
    else
      timer->CCER &= ~(1 << (TIM_CCER_CC1E + (4 * i)));

    // Load the times when interrupts happen
    *ccr_reg[i] = cfg->ccr[i];

    // Set interrupt if required
    timer->DIER &= ~(1 << (TIM_DIER_CC1IE_Pos + i));
    if (cfg->interrupt_en[i] == TIMER_INTERRUPT_ENABLE) timer->DIER |= (1 << (TIM_DIER_CC1IE_Pos + i));
  }

  // Lastly, enable the timer
  timer->CR1 |= (1 << TIM_CR1_CEN_Pos);

  return 0;
}

int timer_set_pwm(TIM_TypeDef *timer, const uint8_t channel, const uint16_t val) {
  if (timer == NULL) return -1;
  volatile uint32_t *ccr_reg[] = TIMERS_CCR_REGS(timer);
  *ccr_reg[(int)channel] = val;
  return 0;
}

void timer_irq_interrupt_config(const uint8_t irq_number, const uint8_t en_state) {
  // Enables or disables NVIC
  // Programs ISER if enable, programs ICER if disable
  if (en_state == TIMER_IRQ_ENABLE)
    NVIC->ISER[irq_number / 32] |= (1 << (irq_number % 32));
  else
    NVIC->ICER[irq_number / 32] |= (1 << (irq_number % 32));
}

void timer_irq_priority_config(const uint8_t irq_number, const uint8_t irq_priority) {
  uint8_t qshift = (irq_number * 8) % 32;  // Will result in bits 0 - 240*8
  uint8_t qindex = (irq_number * 8) / 32;

  NVIC->IP[qindex] &= ~(0xFF << qshift);
  NVIC->IP[qindex] |= (irq_priority << qshift << 4) & (0xF0 << qshift);
}

int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel) {
  if (timer == NULL) return 0;

  const uint8_t status_regs[] = {TIM_SR_CC1IF_Pos, TIM_SR_CC2IF_Pos, TIM_SR_CC3IF_Pos, TIM_SR_CC4IF_Pos};

  if (timer->SR & (1 << status_regs[channel])) {
    timer->SR &= ~(1 << status_regs[channel]);
    return 1;
  }

  return 0;
}
