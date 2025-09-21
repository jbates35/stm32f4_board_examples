#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"
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

#define SIZEOF(arr) ((unsigned int)sizeof(arr) / sizeof(arr[0]))

#define FAST 100000
#define MEDIUM 300000
#define SLOW 1000000
#define WAIT(CNT)                                          \
  do {                                                     \
    for (int sleep_cnt = 0; sleep_cnt < CNT; sleep_cnt++); \
  } while (0)

typedef struct {
  int16_t accel_x;    // Accelerometer x
  int16_t accel_y;    // Accelerometer y
  int16_t accel_z;    // Accelerometer z
  int16_t gyro_rx;    // Gyroscope rx
  int16_t gyro_ry;    // Gyroscope ry
  int16_t gyro_rz;    // Gyroscope rz
  float temperature;  // Temperature
} MPU6050Data;

static MPU6050Data convert_gyro_data(const uint8_t *buf) {
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

void i2c_int_setup();
void i2c_timer_setup();

void main_rx_cb(void);
void setup_rx_cb(void);

uint8_t tx_byte = 0x3B;
uint8_t rx_buff[14] = {0x0};

void main_rx_cb(void) {
  MPU6050Data mpu_data = convert_gyro_data(rx_buff);
  printf("Accel x: %d\n", mpu_data.accel_x);
  printf("Accel y: %d\n", mpu_data.accel_y);
  printf("Accel z: %d\n", mpu_data.accel_z);
  printf("Gyro rx: %d\n", mpu_data.gyro_rx);
  printf("Gyro ry: %d\n", mpu_data.gyro_ry);
  printf("Gyro rz: %d\n", mpu_data.gyro_rz);
  printf("MPU temperature: %f\n\n", mpu_data.temperature);

  // ENABLE TIMER INTERRUPT (ONE-SHOT)
  timer_enable(TIM8);
}

void setup_rx_cb(void) {
  uint8_t gyro_addr = 0x68;

  printf("Setup i2c transition successful\n");

  I2CInterruptConfig_t normal_int_setup = {.tx = {.len = 1, .buff = &tx_byte},
                                           .rx = {.len = SIZEOF(rx_buff), .buff = rx_buff},
                                           .callback = main_rx_cb,
                                           .address = gyro_addr,
                                           .circular = I2C_INTERRUPT_NON_CIRCULAR};

  i2c_setup_interrupt(I2C1, &normal_int_setup);
  i2c_start_interrupt(I2C1);
}

int main(void) {
  i2c_timer_setup();
  i2c_int_setup();

  uint8_t gyro_addr = 0x68;
  uint8_t wake_mpu[] = {0x6B, 0x00};

  WAIT(FAST);

  printf("Starting...\n\n");

  I2CInterruptConfig_t startup_int_setup = {.tx = {.len = SIZEOF(wake_mpu), .buff = wake_mpu},
                                            .rx = {.len = 0},
                                            .callback = setup_rx_cb,
                                            .address = gyro_addr,
                                            .circular = I2C_INTERRUPT_NON_CIRCULAR};
  i2c_setup_interrupt(I2C1, &startup_int_setup);
  i2c_start_interrupt(I2C1);

  for (;;) {
    WAIT(MEDIUM);
  }
}

void I2C1_EV_IRQHandler(void) { i2c_irq_word_handling(I2C1); }

void I2C1_ER_IRQHandler(void) {
  I2CIRQType_t irq_error = i2c_irq_error_handling(I2C1);

  if (irq_error == I2C_IRQ_TYPE_ERROR_ACKFAIL) {
    i2c_reset_interrupt(I2C1);
    i2c_start_interrupt(I2C1);
  }
}

void TIM8_CC_IRQHandler(void) {
  if (timer_irq_handling(TIM8, 1)) {
    i2c_reset_interrupt(I2C1);
    i2c_start_interrupt(I2C1);
  }
}

void i2c_int_setup() {
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
