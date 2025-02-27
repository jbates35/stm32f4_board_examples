// Will use as backup for main for now
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

// NOTE:
// Where I am at right now:
// -DMA for some reason faults out with the FIFO buffer. This does not happen with it during ADC.
// -I can only send a message once. After sending one message, I cannot re-enable the DMA stream.
// -In circular mode, I can make an niterrupt which stops the DMA stream until we're ready again. THis seems to work okay.

#include "main.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define SPI_PORT SPI1
#define SPI_BAUD_RATE 0b10
#define SPI_GPIO_PORT GPIOA
#define SPI_GPIO_NSS_PIN 4
#define SPI_GPIO_CLK_PIN 5
#define SPI_GPIO_MISO_PIN 6
#define SPI_GPIO_MOSI_PIN 7

#define DMA_SPI_STREAM DMA2_Stream5

void spi_master_setup_test();
void spi_tx_in_for_loop();

int spi_tx_byte(SPI_TypeDef *spi_port, const uint16_t tx_byte);
int spi_tx_word(SPI_TypeDef *spi_port, const uint8_t *tx_buffer, uint16_t len);

int spi_rx_byte(SPI_TypeDef *spi_port, uint16_t *rx_byte);
int spi_rx_word(SPI_TypeDef *spi_port, uint8_t *rx_buffer, uint16_t len);

void spi_master_setup_dma_test(char *in_arr, uint16_t elements);
void spi_master_dma_exti_handler();

char dma_tx_str[17];

int main(void) {
  setup_gpio();
  spi_master_setup_test();

  for (;;) {
  }
}

void EXTI15_10_IRQHandler(void) {
  static uint8_t method_num = 0;

  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    char led_on[] = "P 91";
    char led_off[] = "P 90";
    char sensor_read[] = "Q0";

    uint8_t rx_word[255];
    memset(&rx_word, '\0', 255);

    spi_rx_word(SPI1, (const uint8_t *)rx_word, 5);
    // switch (method_num) {
    //   case 0:
    //     spi_tx_word(SPI1, (const uint8_t *)led_on, SIZEOF(led_on) - 1);
    //     spi_rx_word(SPI1, (const uint8_t *)rx_word, 1);
    //     break;
    //   case 1:
    //     // spi_tx_word(SPI1, (const uint8_t *)sensor_read, SIZEOF(sensor_read) - 1);
    //     break;
    //   case 2:
    //     spi_tx_word(SPI1, (const uint8_t *)led_off, SIZEOF(led_off) - 1);
    //     spi_rx_word(SPI1, (const uint8_t *)rx_word, 1);
    //     break;
    // }
    // printf("%d", (int)rx_word[0]);

    method_num++;
    method_num = method_num % 3;
  }
}

void spi_master_setup_test() {
  // The configuration procedure is almost the same for master and slave. For specific mode setups, follow the dedicated chapters. When a standard communication is to be initialized, perform these steps:

  // First enable RCC clock
  RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI1EN_Pos);

  // 1.Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
  GPIO_peri_clock_control(SPI_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOConfig_t default_gpio_cfg = {.mode = GPIO_MODE_ALTFN,
                                   .speed = GPIO_SPEED_HIGH,
                                   .float_resistor = GPIO_PUPDR_NONE,
                                   .output_type = GPIO_OP_TYPE_PUSHPULL,
                                   .alt_func_num = 5};
  GPIOHandle_t spi_gpio_clk_handle = {.p_GPIO_addr = SPI_GPIO_PORT, .cfg = default_gpio_cfg};
  spi_gpio_clk_handle.cfg.pin_number = SPI_GPIO_CLK_PIN;
  GPIO_init(&spi_gpio_clk_handle);

  GPIOHandle_t spi_gpio_miso_handle = {.p_GPIO_addr = SPI_GPIO_PORT, .cfg = default_gpio_cfg};
  spi_gpio_miso_handle.cfg.pin_number = SPI_GPIO_MISO_PIN;
  GPIO_init(&spi_gpio_miso_handle);

  GPIOHandle_t spi_gpio_mosi_handle = {.p_GPIO_addr = SPI_GPIO_PORT, .cfg = default_gpio_cfg};
  spi_gpio_mosi_handle.cfg.pin_number = SPI_GPIO_MOSI_PIN;
  GPIO_init(&spi_gpio_mosi_handle);

  GPIOHandle_t spi_gpio_nss_handle = {.p_GPIO_addr = SPI_GPIO_PORT, .cfg = default_gpio_cfg};
  spi_gpio_nss_handle.cfg.pin_number = SPI_GPIO_NSS_PIN;
  GPIO_init(&spi_gpio_nss_handle);

  // 2.Write to the SPI_CR1 register:
  // a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 3).
  // NOTE says: These bits should not be changed when communication is ongoing.
  SPI_PORT->CR1 |= (SPI_BAUD_RATE << SPI_CR1_BR_Pos);

  // b) Configure the CPOL and CPHA bits combination to define one of the fou  relationships between the data transfer and the serial clock. (Note: 2 - except the  case when CRC is enabled at TI mode).
  SPI_PORT->CR1 |= (0 << SPI_CR1_CPOL_Pos);
  SPI_PORT->CR1 |= (0 << SPI_CR1_CPHA_Pos);

  // c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and BIDIOE (RXONLY and BIDIMODE can't be set at the same time).
  SPI_PORT->CR1 |= (0 << SPI_CR1_BIDIMODE_Pos);
  // BIDIOE is bidirectional output enable (output-only on the pin, as opposed to input-only)

  // d) Configure the LSBFIRST bit to define the frame format (Note: 2).
  SPI_PORT->CR1 |= (0 << SPI_CR1_LSBFIRST_Pos);  // MSB first

  // e) Configure the CRCEN and CRCEN bits if CRC is needed (while SCK clock signal is at idle state
  SPI_PORT->CR1 |= (0 << SPI_CR1_CRCEN_Pos);

  // f) Configure SSM and SSI (Note: 2).
  SPI_PORT->CR1 |= (0 << SPI_CR1_SSM_Pos);
  SPI_PORT->CR1 |= (0 << SPI_CR1_SSI_Pos);

  // g) Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on NSS if master is configured to prevent MODF error).
  SPI_PORT->CR1 |= (1 << SPI_CR1_MSTR_Pos);

  // Ad hoc - turn software slave management on

  // h) Set the DFF bit to configure the data frame format (8 or 16 bits).
  SPI_PORT->CR1 |= (0 << SPI_CR1_DFF_Pos);  // 8-bit DFFs

  // 3:Write to SPI_CR2 register:
  //    a) Configure SSOE (Note: 1 & 2).
  SPI_PORT->CR2 |= (1 << SPI_CR2_SSOE_Pos);

  //    b) Set the FRF bit if the TI protocol is required. Motorola vs TI
  SPI_PORT->CR2 |= (0 << SPI_CR2_FRF_Pos);

  // 4.Write to SPI_CRCPR register: Configure the CRC polynomial if needed.

  // 5.Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in DMA registers if the DMA streams are used.

  // Lastly, enable the SPI peripheral
  SPI_PORT->CR1 |= (1 << SPI_CR1_SPE_Pos);

  // NOTES: Seems like I can only enable master mode when SSI and SSM are 1
  // Otherwise, master gets forced to 0, and MODE FAULT turns to 1

  // REASON: According to the manual, if NSS is pulled low, MODF is set and master mode is cleared
  // Therefore it is recommended to have a pullup resistor if NSS pin is being used
  // Or have SSOE enabled, which drives the NSS signal low and only allows for one slave device
}

int spi_tx_byte(SPI_TypeDef *spi_port, const uint16_t tx_byte) {
  if (spi_port == NULL) return -1;

  // While the TX Buffer is not empty...
  while (!(spi_port->SR & (1 << SPI_SR_TXE_Pos)));
  spi_port->DR = tx_byte;

  return 0;
}

int spi_tx_word(SPI_TypeDef *spi_port, const uint8_t *tx_buffer, uint16_t len) {
  if (spi_port == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_port->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  while (len > 0) {
    // Get the next frame available
    uint16_t tx_word = 0;
    if (dff_bytes == 1)
      tx_word = *((uint8_t *)tx_buffer);
    else {
      tx_word = *((uint16_t *)tx_buffer);
      if (len == 1) tx_word &= 0xFF00;
    }

    // Send byte
    int result = spi_tx_byte(spi_port, tx_word);
    if (result != 0) return result;

    // Move buffer along to the next available frame
    tx_buffer += dff_bytes;
    len -= dff_bytes;
  }

  return 0;
}

void spi_tx_in_for_loop() {
  setup_gpio();
  spi_master_setup_test();

  char test_str[] = "WHO LET THE DOwaejfoiwefjiT";
  int len = SIZEOF(test_str);
  for (;;) {
    spi_tx_word(SPI_PORT, (const uint8_t *)&len, 1);
    spi_tx_word(SPI_PORT, (uint8_t *)test_str, len);
    WAIT(FAST);
  }
}

int spi_rx_byte(SPI_TypeDef *spi_port, uint16_t *rx_byte) {
  if (spi_port == NULL) return -1;
  spi_port->DR = 0;  // Dummy bit so clock can activate
  while (!(spi_port->SR & (1 << SPI_SR_RXNE_Pos)));
  *rx_byte = spi_port->DR;
  return 0;
}

int spi_rx_word(SPI_TypeDef *spi_port, uint8_t *rx_buffer, uint16_t len) {
  if (spi_port == NULL) return -1;

  // Get the amount of bytes per frame - Should be 1 bytes, or 2 bytes (dff=1)
  uint8_t dff_bytes = ((spi_port->CR1 >> SPI_CR1_DFF_Pos) & 0b1) + 1;

  while (len > 0) {
    uint16_t rx_byte = 0;
    spi_rx_byte(spi_port, &rx_byte);

    uint8_t bytes_given;
    if (dff_bytes == 1 || len == 1) {
      *((uint8_t *)rx_buffer) = rx_byte & 0xFF;
      bytes_given = 1;
    } else {
      *((uint16_t *)rx_buffer) = rx_byte;
      bytes_given = 2;
    }

    // Move buffer along to the next available frame
    rx_buffer += bytes_given;
    len -= bytes_given;
  }

  return 0;
}

void spi_rx_in_for_loop() {
  setup_gpio();
  spi_master_setup_test();

  char test_str[] = "WHO LET THE DOwaejfoiwefjiT";
  int len = SIZEOF(test_str);
  for (;;) {
    // uint16_t rx_byte = 0;
    // while (!(SPI_PORT->SR & (1 << SPI_SR_RXNE_Pos)));
    // rx_byte = SPI_PORT->DR;
    // ITM_SendChar(rx_byte);
  }
}

void spi_master_setup_dma_test(char *in_arr, uint16_t elements) {
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t spi_dma_tx_handle = {
      .cfg = {.in = {.addr = (uintptr_t)in_arr, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .out = {.addr = (uintptr_t)&SPI_PORT->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .mem_data_size = DMA_DATA_SIZE_8_BIT,
              .peri_data_size = DMA_DATA_SIZE_8_BIT,
              .dma_elements = elements,
              .channel = 3,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_FINITE,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL,
              .interrupt_en =
                  {
                      .direct_mode_error = DMA_INTERRUPT_DISABLE,
                      .transfer_error = DMA_INTERRUPT_DISABLE,
                      .full_transfer = DMA_INTERRUPT_DISABLE,
                      .half_transfer = DMA_INTERRUPT_DISABLE,
                  },
              .start_enabled = DMA_START_DISABLED},
      .p_stream_addr = DMA_SPI_STREAM};
  dma_stream_init(&spi_dma_tx_handle);

  spi_master_setup_test();

  SPI_PORT->CR1 &= ~(1 << SPI_CR1_SPE_Pos);
  SPI_PORT->CR2 |= (1 << SPI_CR2_TXDMAEN_Pos);

  SPI_PORT->CR1 |= (1 << SPI_CR1_SPE_Pos);
}

void spi_master_dma_exti_handler() {
  if (GPIO_irq_handling(USER_PBUTTON_PIN)) {
    memset(&dma_tx_str, 0, SIZEOF(dma_tx_str));
    char test_str[] = "asdfjkl lkjfdsa";
    int len = SIZEOF(test_str);
    dma_tx_str[0] = (char)len;
    strcat(dma_tx_str, test_str);

    dma_start_transfer(DMA_SPI_STREAM, SIZEOF(dma_tx_str));
  }
}
