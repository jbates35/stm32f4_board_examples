
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_adc.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                          \
  do {                                                     \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++); \
  } while (0)

#define ADC1_CHAN0_GPIO_PORT GPIOA
#define ADC1_CHAN0_GPIO_PIN 0

#define ADC2_CHAN0_GPIO_PORT GPIOA
#define ADC2_CHAN0_GPIO_PIN 1

void adc_button_exti11_setup();
void adc_dual_gpio_setup();
void adc_driver_dual_setup(volatile uint32_t *out_arr, const uint8_t arr_len);

volatile uint32_t dual_var[1];

int main(void) {
  adc_button_exti11_setup();
  adc_dual_gpio_setup();
  adc_driver_dual_setup(dual_var, 1);

  for (;;) {
  }
}

void DMA2_Stream0_IRQHandler(void) {
  if (dma_irq_handling(DMA2, 0, DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE)) {
    uint16_t val1 = dual_var[0] & 0xFFFF;
    uint16_t val2 = (dual_var[0] >> 16) & 0xFFFF;
    uint32_t set_breakpoint_here = (val1 << 16) + val2;
  }
}

void adc_button_exti11_setup() {
  GPIOHandle_t btn_handler = {.cfg =
                                  {
                                      .mode = GPIO_MODE_IT_RT,
                                      .float_resistor = GPIO_PUPDR_PULLDOWN,
                                      .speed = GPIO_SPEED_MEDIUM,
                                      .pin_number = 11,
                                      .alt_func_num = 0,
                                  },
                              .p_GPIO_addr = GPIOD};
  GPIO_peri_clock_control(GPIOD, GPIO_PERI_CLOCK_ENABLE);
  GPIO_init(&btn_handler);
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
}

void adc_driver_dual_setup(volatile uint32_t *out_arr, const uint8_t arr_len) {
  DMAHandle_t adc_dma_handle = {
      .cfg = {.in = {.addr = (uintptr_t)&ADC->CDR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)out_arr, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_32_BIT,
              .peri_data_size = DMA_DATA_SIZE_32_BIT,
              .dma_elements = arr_len,
              .channel = 0b000,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_CIRCULAR,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL,
              .interrupt_en =
                  {
                      .full_transfer = DMA_INTERRUPT_ENABLE,
                      .transfer_error = DMA_INTERRUPT_DISABLE,
                      .direct_mode_error = DMA_INTERRUPT_DISABLE,
                      .half_transfer = DMA_INTERRUPT_DISABLE,
                  },
              .start_enabled = DMA_START_ENABLED},
      .p_stream_addr = DMA2_Stream0};
  dma_peri_clock_control(DMA2, DMA_PERI_CLOCK_ENABLE);
  dma_stream_init(&adc_dma_handle);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  ADCHandle_t adc_dual_handle = {
      .cfg = {.dual_cfg = {.en = ADC_DUAL_MODE_ENABLE, .data_cfg = ADC_DATA_CONFIG_GROUPED},
              .eoc_sel = ADC_INTERRUPT_EOC_SELECT_GROUP,
              .interrupt_en = ADC_INTERRUPT_DISABLE,
              .inj_autostart = ADC_INJ_AUTOSTART_OFF,
              .main_seq_chan_cfg = {.en = ADC_SCAN_ENABLE,
                                    .sequence = {{.channel = 0, .speed = ADC_CHANNEL_SPEED_LOW}},
                                    .sequence_length = 1},
              .main_inj_chan_cfg.en = ADC_SCAN_DISABLE,
              .slave_seq_chan_cfg = {.en = ADC_SCAN_ENABLE,
                                     .sequence = {{.channel = 1, .speed = ADC_CHANNEL_SPEED_LOW}},
                                     .sequence_length = 1},
              .slave_inj_chan_cfg.en = ADC_SCAN_DISABLE,
              .resolution = ADC_RESOLUTION_12_BIT,
              .temp_or_bat_en = ADC_TEMPORBAT_DISABLE,
              .trigger_cfg = {.mode = ADC_TRIGGER_MODE_EXTI11,
                              .edge_sel = ADC_TRIGGER_EDGE_RISING,
                              .channel_type_sel = ADC_TRIGGER_CHANNEL_TYPE_NORMAL}},
      .addr = ADC1};
  adc_peri_clock_control(ADC1, ADC_PERI_CLOCK_ENABLE);
  adc_peri_clock_control(ADC2, ADC_PERI_CLOCK_ENABLE);
  adc_init(&adc_dual_handle);
}
