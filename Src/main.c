#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_usart.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning                                                                       \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int _write(int le, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

#define VERY_FAST 16
#define FAST 100000
#define MEDIUM 300001
#define SLOW 1000000

#define WAIT(CNT)                                                              \
  do {                                                                         \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++)                      \
      ;                                                                        \
  } while (0)

int main(void) {
  for (;;) {
    WAIT(MEDIUM);
  }
}

void setup_uart() {
  USARTConfig_t uart_cfg = {.baud_rate = USART_BAUD_RATE_115200,
                            .peri_clock_freq_hz = 16E6,
                            .en_on_start = USART_ENABLE,
                            .mode = USART_MODE_RX_ONLY,
                            .hw_flow_control = USART_HW_FLOW_NONE,
                            .parity_type = USART_PARITY_NONE,
                            .stop_bit_count = USART_STOP_BITS_ONE,
                            .word_length = USART_WORD_LENGTH_8_BIT_DATA,
                            .synchronous = USART_ASYNCHRONOUS};
}
