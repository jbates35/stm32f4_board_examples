#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"

uint16_t adc_vals2[16];
void dma_adc_setup();

uint16_t dma_arr_1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
uint16_t dma_arr_2[10];
void dma_mem_to_mem_setup();

void dma_adc_setup() {
  // In this one, I believe we have to enable DMA mode, otherwise we simply cannot access any channel except the first one

  // // Enable DMA clock
  // RCC->AHB1ENR |= (1 << RCC_AHB1ENR_DMA2EN_Pos);
  //
  // // Prep DMA for ADC
  // // Set channel 0 for stream 0 for ADC1
  // DMA2_Stream0->CR &= ~(0b111 << DMA_SxCR_CHSEL_Pos);
  // DMA2_Stream0->CR |= (0b000 << DMA_SxCR_CHSEL_Pos);
  //
  // // Set priority level high
  // DMA2_Stream0->CR &= ~(0b11 << DMA_SxCR_PL_Pos);
  // DMA2_Stream0->CR |= (0b11 << DMA_SxCR_PL_Pos);
  //
  // // Enable 12-bit ADC word size in DMA
  // DMA2_Stream0->CR &= ~(0b11 << DMA_SxCR_PSIZE_Pos);
  // DMA2_Stream0->CR |= (0b01 << DMA_SxCR_PSIZE_Pos);
  //
  // // Set memory data size to 16-bit
  // DMA2_Stream0->CR &= ~(0b11 << DMA_SxCR_MSIZE_Pos);
  // DMA2_Stream0->CR |= (0b01 << DMA_SxCR_MSIZE_Pos);
  //
  // // Peripheral increment needs to be turned off
  // DMA2_Stream0->CR &= ~(1 << DMA_SxCR_PINC_Pos);
  //
  // // Memory increment needs to be turned on
  // DMA2_Stream0->CR |= (1 << DMA_SxCR_MINC_Pos);
  //
  // // Assign the DMA peripheral address register to the ADC data register
  // DMA2_Stream0->PAR = (uint32_t)(uintptr_t)&ADC1->DR;
  //
  // // Allow for enough words to be stored in the DMA buffer
  // DMA2_Stream0->NDTR = 16;
  //
  // // Store the words in a buffer
  // DMA2_Stream0->M0AR = (uint32_t)(uintptr_t)adc_vals2;
  //
  // // Turn on DMA
  // DMA2_Stream0->CR |= (1 << DMA_SxCR_EN_Pos);

  //
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t adc_dma_handle = {
      .cfg =
          {
              .in = {.addr = (uintptr_t)&ADC1->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = (uintptr_t)&adc_vals2, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_16_BIT,
              .peri_data_size = DMA_DATA_SIZE_16_BIT,
              .dma_elements = 3,
              .channel = 0b000,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_CIRCULAR,

          },
      .p_stream_addr = DMA2_Stream0};
  dma_stream_init(&adc_dma_handle);
}

void dma_mem_to_mem_setup() {
  dma_peri_clock_control(DMA2, 1);
  DMAHandle_t adc_dma_handle = {
      .cfg =
          {
              .in = {.addr = (uintptr_t)dma_arr_1, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .out = {.addr = (uintptr_t)dma_arr_2, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_16_BIT,
              .peri_data_size = DMA_DATA_SIZE_16_BIT,
              .dma_elements = 10,
              .priority = DMA_PRIORITY_HIGH,
              .circ_buffer = DMA_BUFFER_CIRCULAR,

          },
      .p_stream_addr = DMA2_Stream1};
  dma_stream_init(&adc_dma_handle);
}
