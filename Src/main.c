#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_dma.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_i2c.h"
#include "stm32f446xx_tim.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int _write(int le, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}

#define I2C_GPIO_PORT GPIOB
#define I2C_GPIO_SCL_PIN 8
#define I2C_GPIO_SDA_PIN 9

#define I2C_PORT I2C1

#define I2C_DMA_TX_PORT DMA1
#define I2C_DMA_TX_STREAM DMA1_Stream6
#define I2C_DMA_TX_CHANNEL 1
#define I2C_DMA_TX_STREAM_IRQN DMA1_Stream6_IRQn
#define I2C_DMA_TX_STREAM_IRQ_HANDLER DMA1_Stream6_IRQHandler

#define I2C_DMA_RX_PORT DMA1
#define I2C_DMA_RX_STREAM DMA1_Stream5
#define I2C_DMA_RX_CHANNEL 1
#define I2C_DMA_RX_STREAM_IRQN DMA1_Stream5_IRQn
#define I2C_DMA_RX_STREAM_IRQ_HANDLER DMA1_Stream5_IRQHandler

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

#define VERY_FAST 16
#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                          \
  do {                                                     \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++); \
  } while (0)

void i2c_dma_setup();
void i2c_timer_setup();
void setup_start_sequence_dma(void);
void setup_main_sequence_dma(void);
void express_data(void);

uint8_t gyro_addr = 0x68;
uint8_t tx_start_buff[2] = {0x6B, 0x0};
uint8_t tx_buff[1] = {0x3B};
uint8_t rx_buff[14] = {0x0};

typedef struct {
  int16_t accel_x;    // Accelerometer x
  int16_t accel_y;    // Accelerometer y
  int16_t accel_z;    // Accelerometer z
  int16_t gyro_rx;    // Gyroscope rx
  int16_t gyro_ry;    // Gyroscope ry
  int16_t gyro_rz;    // Gyroscope rz
  float temperature;  // Temperature
} MPU6050Data;

static inline MPU6050Data convert_gyro_data(const uint8_t *buf) {
  MPU6050Data ret;

  ret.accel_x = (uint16_t)(buf[0] << 8) | buf[1];
  ret.accel_y = (uint16_t)(buf[2] << 8) | buf[3];
  ret.accel_z = (uint16_t)(buf[4] << 8) | buf[5];
  ret.gyro_rx = (uint16_t)(buf[8] << 8) | buf[9];
  ret.gyro_ry = (uint16_t)(buf[10] << 8) | buf[11];
  ret.gyro_rz = (uint16_t)(buf[12] << 8) | buf[13];

  int16_t temp = (int16_t)((buf[6] << 8) | buf[7]) - 128;
  ret.temperature = (float)temp / 340 + 36.53;

  return ret;
}

void setup_start_sequence_dma(void) {
  I2CDMAConfig_t dma_config = {.address = gyro_addr,
                               .tx = {.buff = tx_start_buff, .len = SIZEOF(tx_start_buff)},
                               .rx = {.buff = NULL, .len = 0},
                               .tx_stream = I2C_DMA_TX_STREAM,
                               .rx_stream = I2C_DMA_RX_STREAM,
                               .dma_set_buffer_cb = dma_set_buffer,
                               .dma_start_transfer_cb = dma_start_transfer,
                               .circular = I2C_INTERRUPT_NON_CIRCULAR,
                               .callback = setup_main_sequence_dma};
  i2c_setup_interrupt_dma(I2C_PORT, &dma_config);
}

void setup_main_sequence_dma(void) {
  I2CDMAConfig_t dma_config = {.address = gyro_addr,
                               .tx = {.buff = tx_buff, .len = SIZEOF(tx_buff)},
                               .rx = {.buff = rx_buff, .len = SIZEOF(rx_buff)},
                               .tx_stream = I2C_DMA_TX_STREAM,
                               .rx_stream = I2C_DMA_RX_STREAM,
                               .dma_set_buffer_cb = dma_set_buffer,
                               .dma_start_transfer_cb = dma_start_transfer,
                               .circular = I2C_INTERRUPT_NON_CIRCULAR,
                               .callback = express_data};
  i2c_setup_interrupt_dma(I2C_PORT, &dma_config);
  timer_enable(TIM8);
}

void express_data(void) {
  MPU6050Data mpu_data = convert_gyro_data(rx_buff);
  printf("Accel x: %d\n", mpu_data.accel_x);
  printf("Accel y: %d\n", mpu_data.accel_y);
  printf("Accel z: %d\n", mpu_data.accel_z);
  printf("Gyro rx: %d\n", mpu_data.gyro_rx);
  printf("Gyro ry: %d\n", mpu_data.gyro_ry);
  printf("Gyro rz: %d\n", mpu_data.gyro_rz);
  printf("MPU temperature: %f\n\n", mpu_data.temperature);
  timer_enable(TIM8);
}

int main(void) {
  GPIO_i2c_bus_reset(I2C_GPIO_PORT, I2C_GPIO_SCL_PIN);

  WAIT(FAST);

  i2c_timer_setup();
  i2c_dma_setup();

  uint8_t gyro_addr = 0x68;
  uint8_t wake_mpu[] = {0x6B, 0x00};

  printf("Assigning DMA\n");
  setup_start_sequence_dma();

  printf("Starting...\n\n");
  i2c_start_interrupt_dma(I2C_PORT);

  for (;;) {
    WAIT(MEDIUM);
  }
}

void I2C1_EV_IRQHandler(void) { i2c_dma_irq_handling_start(I2C_PORT); }

void I2C1_ER_IRQHandler(void) {
  I2CIRQType_t irq_error = i2c_irq_error_handling(I2C1);
  if (irq_error == I2C_IRQ_TYPE_ERROR_ACKFAIL) {
    printf("Error...\n");
    i2c_start_interrupt_dma(I2C1);
  }
}

void TIM8_CC_IRQHandler(void) {
  int ajfoweijafowjoiawf = 0;
  if (timer_irq_handling(TIM8, 1)) {
    i2c_start_interrupt_dma(I2C_PORT);
  }
}

void I2C_DMA_TX_STREAM_IRQ_HANDLER(void) {
  if (dma_irq_handling(I2C_DMA_TX_STREAM, DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE))
    i2c_dma_irq_handling_end(I2C_PORT, I2C_TXRX_DIR_SEND);
}

void I2C_DMA_RX_STREAM_IRQ_HANDLER(void) {
  if (dma_irq_handling(I2C_DMA_RX_STREAM, DMA_INTERRUPT_TYPE_FULL_TRANSFER_COMPLETE))
    i2c_dma_irq_handling_end(I2C_PORT, I2C_TXRX_DIR_RECEIVE);
}

void i2c_dma_setup() {
  GPIO_peri_clock_control(I2C_GPIO_PORT, GPIO_CLOCK_ENABLE);
  GPIOConfig_t default_gpio_cfg = {.mode = GPIO_MODE_ALTFN,
                                   .speed = GPIO_SPEED_MEDIUM,
                                   .float_resistor = GPIO_PUPDR_NONE,
                                   .output_type = GPIO_OP_TYPE_OPENDRAIN,
                                   .alt_func_num = 4};

  GPIOHandle_t i2c_sda_handle = {.p_GPIO_addr = I2C_GPIO_PORT, .cfg = default_gpio_cfg};
  i2c_sda_handle.cfg.pin_number = I2C_GPIO_SDA_PIN;
  GPIO_init(&i2c_sda_handle);

  GPIOHandle_t i2c_scl_handle = {.p_GPIO_addr = I2C_GPIO_PORT, .cfg = default_gpio_cfg};
  i2c_scl_handle.cfg.pin_number = I2C_GPIO_SCL_PIN;
  GPIO_init(&i2c_scl_handle);

  dma_peri_clock_control(I2C_DMA_TX_PORT, DMA_ENABLE);
  DMAHandle_t dma_tx_handle = {
      .stream_addr = I2C_DMA_TX_STREAM,
      .cfg = {.in = {.addr = NULL, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .out = {.addr = &I2C_PORT->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .mem_data_size = DMA_DATA_SIZE_8_BIT,
              .peri_data_size = DMA_DATA_SIZE_8_BIT,
              .dma_elements = 0,
              .channel = I2C_DMA_TX_CHANNEL,
              .priority = DMA_PRIORITY_HIGH,
              .circ_buffer = DMA_BUFFER_FINITE,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL,
              .interrupt_en =
                  {
                      .direct_mode_error = DMA_DISABLE,
                      .transfer_error = DMA_DISABLE,
                      .full_transfer = DMA_ENABLE,
                      .half_transfer = DMA_DISABLE,
                  },
              .start_enabled = DMA_DISABLE},
  };
  dma_stream_init(&dma_tx_handle);
  NVIC_EnableIRQ(I2C_DMA_TX_STREAM_IRQN);

  dma_peri_clock_control(I2C_DMA_RX_PORT, DMA_ENABLE);
  DMAHandle_t i2c_dma_rx_handle = {
      .cfg = {.in = {.addr = &I2C_PORT->DR, .type = DMA_IO_TYPE_PERIPHERAL, .inc = DMA_IO_ARR_STATIC},
              .out = {.addr = NULL, .type = DMA_IO_TYPE_MEMORY, .inc = DMA_IO_ARR_INCREMENT},
              .mem_data_size = DMA_DATA_SIZE_8_BIT,
              .peri_data_size = DMA_DATA_SIZE_8_BIT,
              .dma_elements = 0,
              .channel = I2C_DMA_RX_CHANNEL,
              .priority = DMA_PRIORITY_MAX,
              .circ_buffer = DMA_BUFFER_FINITE,
              .flow_control = DMA_PERIPH_NO_FLOW_CONTROL,
              .interrupt_en =
                  {
                      .direct_mode_error = DMA_DISABLE,
                      .transfer_error = DMA_DISABLE,
                      .full_transfer = DMA_ENABLE,
                      .half_transfer = DMA_DISABLE,
                  },
              .start_enabled = DMA_DISABLE},
      .stream_addr = I2C_DMA_RX_STREAM};
  dma_stream_init(&i2c_dma_rx_handle);
  NVIC_EnableIRQ(I2C_DMA_RX_STREAM_IRQN);

  i2c_peri_clock_control(I2C_PORT, I2C_ENABLE);
  I2CHandle_t i2c_handle = {.addr = I2C_PORT,
                            .cfg = {.peri_clock_freq_hz = (uint32_t)16E6,
                                    .device_mode = I2C_DEVICE_MODE_MASTER,
                                    .scl_mode = I2C_SCL_MODE_SPEED_SM,
                                    .interrupt_enable = I2C_ENABLE,
                                    .dma_enable = I2C_DISABLE,
                                    .enable_on_init = I2C_ENABLE}};
  i2c_init(&i2c_handle);
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void i2c_timer_setup() {
  TimerHandle_t i2c_tim_handle = {.cfg = {.channel_1 =
                                              {
                                                  .channel_mode = TIMER_CHANNEL_MODE_COMPARE,
                                                  .gpio_en = TIMER_DISABLE,
                                                  .ccr = 0xFFFF,
                                                  .interrupt_en = TIMER_ENABLE,
                                              },
                                          .one_shot_enabled = TIMER_ENABLE,
                                          .start_enabled = TIMER_DISABLE,
                                          .channel_count = 1,
                                          .direction = TIMER_DIR_UP,
                                          .arr = 0xFFFF,
                                          .prescaler = 60},
                                  .p_base_addr = TIM8};
  timer_peri_clock_control(TIM8, 1);
  timer_init(&i2c_tim_handle);
  NVIC_EnableIRQ(TIM8_CC_IRQn);
}
