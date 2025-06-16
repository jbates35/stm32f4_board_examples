
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define SPI_PORT SPI1

#define SPI_GPIO_PORT GPIOA
#define SPI_GPIO_CLK_PIN 5
#define SPI_GPIO_MISO_PIN 6
#define SPI_GPIO_MOSI_PIN 7

#define SPI_GPIO_NSS_PORT GPIOA
#define SPI_GPIO_NSS_PIN 4

#define DMA_SPI_TX_STREAM DMA2_Stream3
#define DMA_SPI_RX_STREAM DMA2_Stream2

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                          \
  do {                                                     \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++); \
  } while (0)


void spi_dma_driver_setup_master(uint8_t *in_arr, uint8_t *out_arr, uint16_t elements);

int main(void) {

  uint8_t mcp3008_dma_tx[3] = {1, (1 << 7) | (1 << 4), 0};
  uint8_t mcp3008_dma_rx[3];

  spi_dma_driver_setup_master(mcp3008_dma_tx, mcp3008_dma_rx, 3);

  for (;;) {
    dma_start_transfer(DMA_SPI_RX_STREAM, 3);
    GPIO_set_output(SPI_GPIO_NSS_PORT, SPI_GPIO_NSS_PIN, 0);
    dma_start_transfer(DMA_SPI_TX_STREAM, 3);
    WAIT(FAST);
  }
}

void DMA2_Stream2_IRQHandler(void) {
  if (dma_irq_handling(DMA_SPI_RX_STREAM, DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE)) {
    uint8_t *rx_arr = (uint8_t *) (DMA_SPI_RX_STREAM->M0AR);

    GPIO_set_output(SPI_GPIO_NSS_PORT, SPI_GPIO_NSS_PIN, 1);
    uint16_t adc_val = ((rx_arr[1] & 3) << 8) | rx_arr[2];
    int breakpoint_set_here = 0;
  }
}

void spi_dma_driver_setup_master(uint8_t *tx_arr, uint8_t *rx_arr, uint16_t elements) {
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t spi_dma_tx_handle = {
      .cfg = {.in = {.addr = (uintptr_t)tx_arr, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
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
      .p_stream_addr = DMA_SPI_TX_STREAM};
  dma_stream_init(&spi_dma_tx_handle);

  DMAHandle_t spi_dma_rx_handle = {
      .cfg = {.in = {.addr = (uintptr_t)&SPI_PORT->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)rx_arr, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_8_BIT,
              .peri_data_size = DMA_DATA_SIZE_8_BIT,
              .dma_elements = elements,
              .channel = 3,
              .priority = DMA_PRIORITY_HIGH,
              .circ_buffer = DMA_BUFFER_FINITE,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL,
              .interrupt_en =
                  {
                      .direct_mode_error = DMA_INTERRUPT_DISABLE,
                      .transfer_error = DMA_INTERRUPT_DISABLE,
                      .full_transfer = DMA_INTERRUPT_ENABLE,
                      .half_transfer = DMA_INTERRUPT_DISABLE,
                  },
              .start_enabled = DMA_START_DISABLED},
      .p_stream_addr = DMA_SPI_RX_STREAM};
  dma_stream_init(&spi_dma_rx_handle);
  dma_peri_clock_control(DMA2, 1);
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  GPIO_peri_clock_control(SPI_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIO_peri_clock_control(SPI_GPIO_NSS_PORT, GPIO_CLOCK_ENABLE);
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

  GPIOHandle_t spi_gpio_nss_handle = {.p_GPIO_addr = SPI_GPIO_NSS_PORT, .cfg = default_gpio_cfg};
  spi_gpio_nss_handle.cfg.pin_number = SPI_GPIO_NSS_PIN;
  spi_gpio_nss_handle.cfg.alt_func_num = 0;
  spi_gpio_nss_handle.cfg.mode = GPIO_MODE_OUT;
  GPIO_init(&spi_gpio_nss_handle);
  GPIO_set_output(SPI_GPIO_NSS_PORT, SPI_GPIO_NSS_PIN, 1);

  spi_peri_clock_control(SPI_PORT, SPI_PERI_CLOCK_ENABLE);
  SPIHandle_t spi_handle = {.addr = SPI_PORT,
                            .cfg = {.baud_divisor = SPI_BAUD_DIVISOR_32,
                                    .bus_config = SPI_BUS_CONFIG_FULL_DUPLEX,
                                    .device_mode = SPI_DEVICE_MODE_MASTER,
                                    .dff = SPI_DFF_8_BIT,
                                    .ssm = SPI_SSM_ENABLE,
                                    .dma_setup = {.rx = SPI_DMA_ENABLE, .tx = SPI_DMA_ENABLE},
                                    .interrupt_setup.en = SPI_INTERRUPT_DISABLE}};
  spi_init(&spi_handle);
}

