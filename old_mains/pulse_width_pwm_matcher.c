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
#include "stm32f446xx_tim.h"

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
#define LED_GREEN_ALT_FN 1

#define USER_PBUTTON_PORT GPIOC
#define USER_PBUTTON_PIN 13

#define INPUT_CAPTURE_GPIO_PORT GPIOB
#define INPUT_CAPTURE_GPIO_PIN 6
#define INPUT_CAPTURE_GPIO_ALT_FN 2

#define PWM_GPIO_PORT GPIOB
#define PWM_GPIO_PIN 3
#define PWM_GPIO_ALT_FN 1

/******* TIMERS ********/
#define TIM_TIMER_ADDR TIM5
#define TIM_CHANNEL 1

#define PWM_TIMER_ADDR TIM2
#define PWM_CHANNEL 2

#define INPUT_CAPTURE_ADDR TIM4
#define INPUT_CAPTURE_CHAN 1
#define OUTPUT_COMPARE_CHAN_LO 2
#define OUTPUT_COMPARE_CHAN_HI 3

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
  // GPIO Instantiation
  // Green LED for PA5 (on nucleo board)
  GPIO_peri_clock_control(LED_GREEN_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t led_green_handler = {.p_GPIO_addr = LED_GREEN_PORT,
                                    .cfg = {.mode = GPIO_MODE_OUT,
                                            .pin_number = LED_GREEN_PIN,
                                            .speed = GPIO_SPEED_LOW,
                                            .output_type = GPIO_OP_TYPE_PUSHPULL,
                                            .float_resistor = GPIO_PUPDR_NONE}};
  GPIO_init(&led_green_handler);

  // PWM Output externally wired to PB3, attached later to timer 2 channel 2
  GPIO_peri_clock_control(PWM_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t pwm_handler = {.p_GPIO_addr = PWM_GPIO_PORT,
                              .cfg = {.mode = GPIO_MODE_ALTFN,
                                      .pin_number = PWM_GPIO_PIN,
                                      .speed = GPIO_SPEED_MEDIUM,
                                      .output_type = GPIO_OP_TYPE_PUSHPULL,
                                      .float_resistor = GPIO_PUPDR_NONE,
                                      .alt_func_num = PWM_GPIO_ALT_FN}};
  GPIO_init(&pwm_handler);

  // User button on PC13, attached to a falling edge interrupt IRQ
  GPIO_peri_clock_control(USER_PBUTTON_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t user_btn_handler = {.p_GPIO_addr = USER_PBUTTON_PORT,
                                   .cfg = {.mode = GPIO_MODE_IT_FT,
                                           .pin_number = USER_PBUTTON_PIN,
                                           .speed = GPIO_SPEED_LOW,
                                           .output_type = GPIO_OP_TYPE_PUSHPULL,
                                           .float_resistor = GPIO_PUPDR_PULLDOWN}};
  GPIO_init(&user_btn_handler);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Input capture on PB6, tied to a timer interrupt which captures the pulse width on timer 4 channel 1
  GPIO_peri_clock_control(INPUT_CAPTURE_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t capture_handler = {.p_GPIO_addr = INPUT_CAPTURE_GPIO_PORT,
                                  .cfg = {.mode = GPIO_MODE_ALTFN,
                                          .pin_number = INPUT_CAPTURE_GPIO_PIN,
                                          .speed = GPIO_SPEED_HIGH,
                                          .output_type = GPIO_OP_TYPE_PUSHPULL,
                                          .float_resistor = GPIO_PUPDR_PULLDOWN,
                                          .alt_func_num = INPUT_CAPTURE_GPIO_ALT_FN}};
  GPIO_init(&capture_handler);

  // For PWM on PB11
  timer_peri_clock_control(TIM2, TIMER_PERI_CLOCK_ENABLE);
  TimerHandle_t tim2_handle = {.p_base_addr = TIM2,
                               .cfg = {.arr = 1000,
                                       .channel_count = 4,
                                       .prescaler = 253,
                                       .channel_2 = {.gpio_en = TIMER_GPIO_ENABLE,
                                                     .interrupt_en = TIMER_INTERRUPT_DISABLE,
                                                     .channel_mode = TIMER_CHANNEL_MODE_PWM_HI,
                                                     .ccr = 0}}};
  timer_init(&tim2_handle);

  // For input capture on PB6 and some compare channels (2&3)
  timer_peri_clock_control(TIM4, TIMER_PERI_CLOCK_ENABLE);
  TimerHandle_t tim4_handle = {
      .p_base_addr = TIM4,
      .cfg = {
          .arr = 0xffff,
          .prescaler = 507,
          .channel_count = 3,
          .channel_1 = {.gpio_en = TIMER_GPIO_ENABLE,
                        .channel_mode = TIMER_CHANNEL_MODE_CAPTURE,
                        .interrupt_en = TIMER_INTERRUPT_ENABLE,
                        .capture_edge = TIMER_CAPTURE_BOTH_EDGE,
                        .capture_input_filter = TIMER_CAPTURE_FILTER_MEDIUM},
          .channel_2 = {.channel_mode = TIMER_CHANNEL_MODE_COMPARE, .interrupt_en = TIMER_INTERRUPT_ENABLE, .ccr = 0},
          .channel_3 = {
              .channel_mode = TIMER_CHANNEL_MODE_COMPARE, .interrupt_en = TIMER_INTERRUPT_ENABLE, .ccr = 0xffff / 4}}};
  timer_init(&tim4_handle);
  NVIC_EnableIRQ(TIM4_IRQn);

  /* Loop forever */
  for (;;) {
  }
}

void TIM4_IRQHandler(void) {
  if (timer_irq_handling(INPUT_CAPTURE_ADDR, INPUT_CAPTURE_CHAN)) {
    uint8_t rising_edge = GPIO_get_input(INPUT_CAPTURE_GPIO_PORT, INPUT_CAPTURE_GPIO_PIN);
    static uint16_t capture_val = 0;
    static uint16_t capture_val_falling = 0;

    // Rising edge
    if (rising_edge) {
      capture_val_falling = timer_get_current_ticks(INPUT_CAPTURE_ADDR, INPUT_CAPTURE_CHAN);
    }
    // Falling edge
    else {
      capture_val = timer_get_current_ticks(INPUT_CAPTURE_ADDR, INPUT_CAPTURE_CHAN) - capture_val_falling;
      float pwm_alpha = (float)capture_val / timer_get_period_ticks(INPUT_CAPTURE_ADDR);
      timer_set_pwm_percent(PWM_TIMER_ADDR, PWM_CHANNEL, pwm_alpha);
    }
  }

  if (timer_irq_handling(INPUT_CAPTURE_ADDR, OUTPUT_COMPARE_CHAN_HI)) {
    GPIO_set_output(LED_GREEN_PORT, LED_GREEN_PIN, 1);
  }

  if (timer_irq_handling(INPUT_CAPTURE_ADDR, OUTPUT_COMPARE_CHAN_LO)) {
    GPIO_set_output(LED_GREEN_PORT, LED_GREEN_PIN, 0);
  }
}

void EXTI15_10_IRQHandler(void) {
  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    timer_set_pwm_percent(PWM_TIMER_ADDR, PWM_CHANNEL, 0);
  }
}
