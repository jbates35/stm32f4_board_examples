#include "stm32f446xx.h"
#include "stm32f446xx_adc.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_tim.h"

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                          \
  do {                                                     \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++); \
  } while (0)

/******* PINS *********/
#define GPIO_GREEN_LED_PORT GPIOC
#define GPIO_GREEN_LED_PIN 0
#define GPIO_BLUE_LED_PORT GPIOB
#define GPIO_BLUE_LED_PIN 0

#define USER_PBUTTON_PORT GPIOC
#define USER_PBUTTON_PIN 13

#define INPUT_CAPTURE_GPIO_PORT GPIOB
#define INPUT_CAPTURE_GPIO_PIN 6
#define INPUT_CAPTURE_GPIO_ALT_FN 2

#define PWM_GPIO_PORT GPIOB
#define PWM_GPIO_PIN 3
#define PWM_GPIO_ALT_FN 1

#define MEASURE_OSC_GPIO_ADDR GPIOB
#define MEASURE_OSC_GPIO_PIN 4
#define MEASURE_OSC_GPIO_ALTFN 2

#define MEASURE_SIG_GPIO_ADDR GPIOB
#define MEASURE_SIG_GPIO_PIN 5

#define ADC1_CHAN0_GPIO_PORT GPIOA
#define ADC1_CHAN0_GPIO_PIN 0

#define ADC1_CHAN1_GPIO_PORT GPIOA
#define ADC1_CHAN1_GPIO_PIN 1

#define ADC2_CHAN0_GPIO_PORT GPIOA
#define ADC2_CHAN0_GPIO_PIN 1

/******* TIMERS ********/
#define TIM_TIMER_ADDR TIM5
#define TIM_CHANNEL 1

#define PWM_TIMER_ADDR TIM2
#define PWM_CHANNEL 2

#define MEASURE_OSC_TIMER_ADDR TIM3
#define MEASURE_OSC_TIMER_CHAN 1

#define INPUT_CAPTURE_ADDR TIM4
#define INPUT_CAPTURE_CHAN 1

#define OUTPUT_COMPARE_CHAN_LO 2
#define OUTPUT_COMPARE_CHAN_HI 3

void adc_gpio_setup();
void adc_test_single_setup();
void adc_test_disc_setup();
void adc_test_scan_setup();
void adc_test_cont_setup();
void adc_dual_gpio_setup();
void adc_dual_channel_setup();

void adc_driver_single_setup();
void adc_tim_scan_example(uint16_t *out_arr, const uint8_t arr_len);
void adc_driver_inj_setup();
void adc_driver_scan_start();

void dma_adc_setup();
void dma_adc_dual_setup();
void read_temperature_setup();
float read_temperature();

int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);

static inline void setup_gpio() {
  GPIOHandle_t led_blue_handler = {.cfg = {.mode = GPIO_MODE_OUT,
                                           .output_type = GPIO_OP_TYPE_PUSHPULL,
                                           .speed = GPIO_SPEED_MEDIUM,
                                           .pin_number = GPIO_BLUE_LED_PIN

                                   },
                                   .p_GPIO_addr = GPIO_BLUE_LED_PORT};
  GPIO_init(&led_blue_handler);
  GPIO_set_output(GPIO_BLUE_LED_PORT, GPIO_BLUE_LED_PIN, 0);

  GPIO_peri_clock_control(GPIO_GREEN_LED_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t led_green_handler = {.p_GPIO_addr = GPIO_GREEN_LED_PORT,
                                    .cfg = {.mode = GPIO_MODE_OUT,
                                            .pin_number = GPIO_GREEN_LED_PIN,
                                            .speed = GPIO_SPEED_LOW,
                                            .output_type = GPIO_OP_TYPE_PUSHPULL}};
  GPIO_init(&led_green_handler);
  GPIO_set_output(GPIO_GREEN_LED_PORT, GPIO_GREEN_LED_PIN, 0);

  // User button on PC13, attached to a falling edge interrupt IRQ
  GPIO_peri_clock_control(USER_PBUTTON_PORT, GPIO_CLOCK_ENABLE);
  GPIOHandle_t user_btn_handler = {.p_GPIO_addr = USER_PBUTTON_PORT,
                                   .cfg = {.mode = GPIO_MODE_IT_FT,
                                           .pin_number = USER_PBUTTON_PIN,
                                           .speed = GPIO_SPEED_LOW,
                                           .float_resistor = GPIO_PUPDR_PULLDOWN}};
  GPIO_init(&user_btn_handler);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static inline void setup_meas_gpio() {
  GPIO_peri_clock_control(MEASURE_OSC_GPIO_ADDR, GPIO_PERI_CLOCK_ENABLE);
  GPIOHandle_t osc_handler = {.p_GPIO_addr = MEASURE_OSC_GPIO_ADDR,
                              .cfg = {.mode = GPIO_MODE_ALTFN,
                                      .pin_number = MEASURE_OSC_GPIO_PIN,
                                      .speed = GPIO_SPEED_MEDIUM,
                                      .output_type = GPIO_OP_TYPE_PUSHPULL,
                                      .alt_func_num = MEASURE_OSC_GPIO_ALTFN}};
  GPIO_init(&osc_handler);

  GPIO_peri_clock_control(MEASURE_SIG_GPIO_ADDR, GPIO_PERI_CLOCK_ENABLE);
  GPIOHandle_t sig_handler = {.p_GPIO_addr = MEASURE_SIG_GPIO_ADDR,
                              .cfg = {
                                  .mode = GPIO_MODE_OUT,
                                  .pin_number = MEASURE_SIG_GPIO_PIN,
                                  .speed = GPIO_SPEED_MEDIUM,
                                  .output_type = GPIO_OP_TYPE_PUSHPULL,
                              }};
  GPIO_init(&sig_handler);
}
int _write(int le, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}
