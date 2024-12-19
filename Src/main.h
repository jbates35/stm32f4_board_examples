#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_tim.h"

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

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

#define SPI_GPIO_PORT GPIOA
#define SPI_GPIO_NSS_PIN 4
#define SPI_GPIO_CLK_PIN 5
#define SPI_GPIO_MISO_PIN 6
#define SPI_GPIO_MOSI_PIN 7

#define ADC_CHAN0_GPIO_PORT GPIOA
#define ADC_CHAN0_GPIO_PIN 0

#define ADC_CHAN1_GPIO_PORT GPIOA
#define ADC_CHAN1_GPIO_PIN 1
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

void adc_test_setup();
void adc_gpio_setup();
void adc_test_single_setup();
void adc_test_cont_setup();
uint16_t adc_sample();
void read_temperature_setup();
float read_temperature();
float convert_adc_to_temperature(uint16_t adc_val, uint8_t adc_res);
