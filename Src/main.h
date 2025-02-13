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

#define SPI_GPIO_PORT GPIOA
#define SPI_GPIO_NSS_PIN 4
#define SPI_GPIO_CLK_PIN 5
#define SPI_GPIO_MISO_PIN 6
#define SPI_GPIO_MOSI_PIN 7

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

#define INPUT_CAPTURE_ADDR TIM4
#define INPUT_CAPTURE_CHAN 1
#define OUTPUT_COMPARE_CHAN_LO 2
#define OUTPUT_COMPARE_CHAN_HI 3

/****** SPI *********/
#define SPI_PORT SPI1
#define SPI_BAUD_RATE 0b010

void setup_gpio();
void setup_timers();

void spi_setup_test();

int spi_tx_byte(SPI_TypeDef *spi_port, const uint16_t tx_byte);
int spi_tx_word(SPI_TypeDef *spi_port, const uint8_t *tx_buffer, uint16_t len);

int spi_rx_byte(SPI_TypeDef *spi_port, uint16_t *rx_byte);
int spi_rx_word(SPI_TypeDef *spi_port, const uint8_t *rx_buffer, uint16_t len);

void adc_gpio_setup();
void adc_test_single_setup();
void adc_test_disc_setup();
void adc_test_scan_setup();
void adc_test_cont_setup();
void adc_dual_gpio_setup();
void adc_dual_channel_setup();

void adc_driver_single_setup();
void adc_driver_scan_setup(uint16_t *out_arr, const uint8_t arr_len);
void adc_driver_inj_setup();
void adc_driver_scan_start();

void dma_adc_setup();
void dma_adc_dual_setup();
void read_temperature_setup();
float read_temperature();

int timer_irq_handling(TIM_TypeDef *timer, const uint8_t channel);
