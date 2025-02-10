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

#include "main.h"

#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_adc.h"
#include "stm32f446xx_dma.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int adc_cnt = 0;

volatile uint16_t adc_vals2[3];
volatile uint32_t adc_vals_dual[16];

int main(void) {
  adc_vals_dual[0] = 4;

  uint16_t adc_arr[3];

  adc_gpio_setup();

  adc_driver_single_setup();
  // adc_driver_scan_setup(adc_arr, (uint8_t)SIZEOF(adc_arr));

  // NVIC_EnableIRQ(ADC_IRQn);
  // adc_interrupt_en(ADC1);

  for (;;) {
    // adc_scan_sample(ADC1, ADC_NON_BLOCKING);
    //
    // WAIT(SLOW);
    // int asdf = 0;

    uint16_t val1 = adc_single_sample(ADC1, 0, ADC_CHANNEL_SPEED_LOW, ADC_BLOCKING);
    WAIT(SLOW);
    uint16_t val2 = adc_single_sample(ADC1, 1, ADC_CHANNEL_SPEED_LOW, ADC_BLOCKING);
    WAIT(SLOW);
    uint16_t val3 = adc_single_sample(ADC1, 18, ADC_CHANNEL_SPEED_LOW, ADC_BLOCKING);
    WAIT(SLOW);
  }
}

// void ADC_IRQHandler(void) {
//   ADC1->SR &= ~(1 << ADC_SR_EOC_Pos);
//   int asdf = 0;
//   adc_cnt++;
// }

void setup_gpio() {
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
}

void adc_gpio_setup() {
  // PA 0 and 1 will be the ADC channels. That relates to ADC channels 0 and 1
  GPIOConfig_t cfg = {.mode = GPIO_MODE_ANALOG, .speed = GPIO_SPEED_MEDIUM, .float_resistor = GPIO_PUPDR_NONE};

  // ADC 0
  GPIO_peri_clock_control(ADC1_CHAN0_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t adc0_handler = {.p_GPIO_addr = ADC1_CHAN0_GPIO_PORT, .cfg = cfg};
  adc0_handler.cfg.pin_number = ADC1_CHAN0_GPIO_PIN;
  GPIO_init(&adc0_handler);

  // ADC 1
  GPIO_peri_clock_control(ADC1_CHAN1_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t adc1_handler = {.p_GPIO_addr = ADC1_CHAN1_GPIO_PORT, .cfg = cfg};
  adc1_handler.cfg.pin_number = ADC1_CHAN1_GPIO_PIN;
  GPIO_init(&adc1_handler);
}

void adc_driver_single_setup() {
  ADCHandle_t adc_init_struct = {.addr = ADC1,
                                 .cfg = {.dual_cfg.en = ADC_DUAL_MODE_DISABLE,
                                         .inj_autostart = ADC_INJ_AUTOSTART_OFF,
                                         .interrupt_en = ADC_INTERRUPT_DISABLE,
                                         .main_inj_chan_cfg.en = ADC_SCAN_DISABLE,
                                         .main_seq_chan_cfg.en = ADC_SCAN_DISABLE,
                                         .temp_or_bat_en = ADC_TEMPORBAT_TEMPERATURE,
                                         .resolution = ADC_RESOLUTION_12_BIT,
                                         .trigger_cfg = ADC_TRIGGER_MODE_MANUAL}};
  adc_peri_clock_control(ADC1, 1);
  adc_init(&adc_init_struct);
}

void adc_driver_scan_setup(uint16_t* out_arr, const uint8_t arr_len) {
  DMAHandle_t adc_dma_handle = {
      .cfg = {.in = {.addr = (uintptr_t)&ADC1->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)out_arr, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_16_BIT,
              .peri_data_size = DMA_DATA_SIZE_16_BIT,
              .dma_elements = arr_len,
              .channel = 0b001,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_CIRCULAR,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL},
      .p_stream_addr = DMA2_Stream0};
  dma_peri_clock_control(DMA2, DMA_PERI_CLOCK_ENABLE);
  dma_stream_init(&adc_dma_handle);

  ADCHandle_t adc_init_struct = {
      .addr = ADC1,
      .cfg = {.dual_cfg.en = ADC_DUAL_MODE_DISABLE,
              .inj_autostart = ADC_INJ_AUTOSTART_OFF,
              .interrupt_en = ADC_INTERRUPT_DISABLE,
              .main_seq_chan_cfg = {.en = ADC_SCAN_ENABLE,
                                    .sequence = {{.channel = 0, .speed = ADC_CHANNEL_SPEED_LOW},
                                                 {.channel = 1, .speed = ADC_CHANNEL_SPEED_LOW},
                                                 {.channel = 18, .speed = ADC_CHANNEL_SPEED_LOW}

                                    },
                                    .sequence_length = 3},
              .main_inj_chan_cfg.en = ADC_SCAN_DISABLE,
              .eoc_sel = ADC_INTERRUPT_EOC_SELECT_GROUP,
              .temp_or_bat_en = ADC_TEMPORBAT_TEMPERATURE,
              .resolution = ADC_RESOLUTION_12_BIT,
              .trigger_cfg = ADC_TRIGGER_MODE_MANUAL}};
  adc_peri_clock_control(ADC1, ADC_PERI_CLOCK_ENABLE);
  adc_init(&adc_init_struct);
}

void adc_test_scan_setup() {
  // Enable RCC clock
  RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos);

  dma_adc_setup();

  // Enable DMA mode
  ADC1->CR2 |= (1 << ADC_CR2_DMA_Pos);
  ADC1->CR2 |= (1 << ADC_CR2_DDS_Pos);

  // Set up ADC in single conversion
  ADC1->CR2 |= (0 << ADC_CR2_CONT_Pos);

  // Set up ADC in scan mode
  ADC1->CR1 |= (1 << ADC_CR1_SCAN_Pos);

  // Ensure the EOC gets triggered after each conversion
  ADC1->CR2 &= ~(1 << ADC_CR2_EOCS_Pos);

  // Select number of channels to sample
  ADC1->SQR1 &= ~(0xF << ADC_SQR1_L_Pos);
  ADC1->SQR1 |= (0b10 << ADC_SQR1_L_Pos);

  // 3.Select ADC1_IN18 input channel.
  ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);
  ADC1->SQR3 |= (1 << ADC_SQR3_SQ2_Pos);
  ADC1->SQR3 |= (18 << ADC_SQR3_SQ3_Pos);

  // 4.Select a sampling time greater than the minimum sampling time specified in the datasheet.
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP0_Pos);
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP1_Pos);
  ADC1->SMPR1 |= (0b010 << ADC_SMPR1_SMP18_Pos);

  // TRY INJECTED CHANNELS /////////

  // END INJECTED CHANNELS ///////

  // Turn ADC on
  ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos);

  // 5.Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor from power down mode
  ADC123_COMMON->CCR |= (1 << ADC_CCR_TSVREFE_Pos);

  WAIT(FAST);
}

void adc_test_cont_setup() {
  // Single conversion mode
  // In Single conversion mode the ADC does one conversion. This mode is started with the
  // CONT bit at 0 by either:
  // setting the SWSTART bit in the ADC_CR2 register (for a regular channel only)

  // Once the conversion of the selected channel is complete:
  // If a regular channel was converted:
  // –The converted data are stored into the 16-bit ADC_DR register
  // –The EOC (end of conversion) flag is set
  // –An interrupt is generated if the EOCIE bit is set
  // If an injected channel was converted:
  // –The converted data are stored into the 16-bit ADC_JDR1 register
  // –The JEOC (end of conversion injected) flag is set
  // –An interrupt is generated if the JEOCIE bit is set
  // Then the ADC stops.

  // NOTE: Try with the scan mode turned off first, then on
  // Enable RCC clock

  RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos);

  dma_adc_setup();

  // Enable DMA mode
  ADC1->CR2 |= (1 << ADC_CR2_DMA_Pos) | (1 << ADC_CR2_DDS_Pos);

  // Enable continuous mode
  ADC1->CR2 |= (1 << ADC_CR2_CONT_Pos);

  // Set up ADC in scan mode
  // ADC1->CR1 |= (1 << ADC_CR1_SCAN_Pos);

  // Ensure the EOC gets triggered after each conversion
  ADC1->CR2 &= ~(1 << ADC_CR2_EOCS_Pos);

  // Select number of channels to sample
  ADC1->SQR1 &= ~(0xF << ADC_SQR1_L_Pos);
  ADC1->SQR1 |= (0b10 << ADC_SQR1_L_Pos);

  // 3.Select ADC1_IN18 input channel.
  ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);
  ADC1->SQR3 |= (1 << ADC_SQR3_SQ2_Pos);
  ADC1->SQR3 |= (18 << ADC_SQR3_SQ3_Pos);

  // 4.Select a sampling time greater than the minimum sampling time specified in the datasheet.
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP0_Pos);
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP1_Pos);
  ADC1->SMPR1 |= (0b010 << ADC_SMPR1_SMP18_Pos);

  // TRY INJECTED CHANNELS /////////

  // END INJECTED CHANNELS ///////

  // Turn ADC on
  ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos);

  // 5.Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor from power down mode
  ADC123_COMMON->CCR |= (1 << ADC_CCR_TSVREFE_Pos);

  WAIT(FAST);
  // Question then, does it sample just the first channel, or all the channels?
}

void adc_test_injected_setup() {
  // NOTE: with JSQL, it starts, say if JL is 3, from 2->4. Not 1->3.
  adc_test_scan_setup();
  ADC1->JSQR |=
      (0b10 << ADC_JSQR_JL_Pos) | (18 << ADC_JSQR_JSQ4_Pos) | (1 << ADC_JSQR_JSQ3_Pos) | (0 << ADC_JSQR_JSQ2_Pos);
}

void adc_dual_gpio_setup() {
  // PA 0 and 1 will be the ADC channels. That relates to ADC channels 0 and 1
  GPIOConfig_t cfg = {.mode = GPIO_MODE_ANALOG, .speed = GPIO_SPEED_HIGH, .float_resistor = GPIO_PUPDR_NONE};

  // ADC1 chan0
  GPIO_peri_clock_control(ADC1_CHAN0_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t adc0_handler = {.p_GPIO_addr = ADC1_CHAN0_GPIO_PORT, .cfg = cfg};
  adc0_handler.cfg.pin_number = ADC1_CHAN0_GPIO_PIN;
  GPIO_init(&adc0_handler);

  // ADC2 chan0
  GPIO_peri_clock_control(ADC2_CHAN0_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t adc1_handler = {.p_GPIO_addr = ADC2_CHAN0_GPIO_PORT, .cfg = cfg};
  adc1_handler.cfg.pin_number = ADC2_CHAN0_GPIO_PIN;
  GPIO_init(&adc1_handler);

  // // ADC1 chan2
  // GPIO_peri_clock_control(ADC1_CHAN2_GPIO_PORT, GPIO_CLOCK_ENABLE);
  // GPIOHandle_t adc0chan2_handler = {.p_GPIO_addr = ADC1_CHAN2_GPIO_PORT, .cfg = cfg};
  // adc0chan2_handler.cfg.pin_number = ADC1_CHAN2_GPIO_PIN;
  // GPIO_init(&adc0chan2_handler);
}
void adc_dual_channel_setup() {
  // Enable RCC clock
  RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos);
  RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC2EN_Pos);

  dma_adc_dual_setup();

  ADC1->CR1 = 0;
  ADC1->CR2 = 0;
  ADC2->CR1 = 0;
  ADC2->CR2 = 0;

  ADC->CCR = 0;
  ADC->CCR |= (0b10 << ADC_CCR_DMA_Pos) | (1 << ADC_CCR_DDS_Pos);
  ADC->CCR |= (0b00110 << ADC_CCR_MULTI_Pos);
  // ADC->CCR |= (0b11 << ADC_CCR_ADCPRE_Pos);

  // Set up ADC in scan mode
  ADC1->CR1 |= (1 << ADC_CR1_SCAN_Pos);

  // Ensure the EOC gets triggered after each conversion
  ADC1->CR2 &= ~(1 << ADC_CR2_EOCS_Pos);
  ADC1->CR2 |= (1 << ADC_CR1_EOCIE_Pos);

  // Select number of channels to sample
  ADC1->SQR1 &= ~(0xF << ADC_SQR1_L_Pos);
  ADC1->SQR1 |= (0b10 << ADC_SQR1_L_Pos);

  // Select number of channels to sample
  ADC2->SQR1 &= ~(0xF << ADC_SQR1_L_Pos);
  ADC2->SQR1 |= (0b10 << ADC_SQR1_L_Pos);

  ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);
  ADC2->SQR3 |= (1 << ADC_SQR3_SQ1_Pos);

  // 4.Select a sampling time greater than the minimum sampling time specified in the datasheet.
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP0_Pos);
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP1_Pos);
  ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP2_Pos);
  ADC2->SMPR2 |= (0b010 << ADC_SMPR2_SMP0_Pos);
  ADC2->SMPR2 |= (0b010 << ADC_SMPR2_SMP1_Pos);

  // Turn ADC on
  ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos);
  ADC2->CR2 |= (1 << ADC_CR2_ADON_Pos);

  // 5.Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor from power down mode
  ADC123_COMMON->CCR |= (1 << ADC_CCR_TSVREFE_Pos);

  WAIT(FAST);
}

void read_temperature_setup() {
  RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos);

  ADC1->CR2 |= (0 << ADC_CR2_CONT_Pos);
  ADC1->CR2 |= (1 << ADC_CR2_ADON_Pos);
  WAIT(FAST);

  ADC1->SQR1 &= ~(0xF << ADC_SQR1_L_Pos);
  ADC1->SQR1 |= (0 << ADC_SQR1_L_Pos);

  // 3.Select ADC1_IN18 input channel.
  ADC1->SQR3 |= (18 << ADC_SQR3_SQ1_Pos);

  // 4.Select a sampling time greater than the minimum sampling time specified in the datasheet.
  ADC1->SMPR1 |= (0b111 << ADC_SMPR1_SMP18_Pos);

  // 5.Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor from power down mode
  ADC123_COMMON->CCR |= (1 << ADC_CCR_TSVREFE_Pos);
  WAIT(FAST);
}

void adc_interrupt_en(ADC_TypeDef* adc_addr) { adc_addr->CR1 |= (1 << ADC_CR1_EOCIE_Pos); }

void dma_adc_setup() {
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t adc_dma_handle = {
      .cfg = {.in = {.addr = (uintptr_t)&ADC1->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)adc_vals2, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_16_BIT,
              .peri_data_size = DMA_DATA_SIZE_16_BIT,
              .dma_elements = 3,
              .channel = 0b000,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_CIRCULAR,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL},
      .p_stream_addr = DMA2_Stream0};
  dma_stream_init(&adc_dma_handle);
}

void dma_adc_dual_setup() {
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t adc_dma_handle = {
      .cfg = {.in = {.addr = (uintptr_t)&ADC->CDR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)adc_vals_dual, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_32_BIT,
              .peri_data_size = DMA_DATA_SIZE_32_BIT,
              .dma_elements = 16,
              .channel = 0b000,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_CIRCULAR,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL},
      .p_stream_addr = DMA2_Stream0};
  dma_stream_init(&adc_dma_handle);
}

// Thigns needed for ADC
// - Dual channel mode
// - Injected channels
// - Calibration
// - Temperature sense
// - Single sample mode, multi-channel
// - Manual sample, auto sample
