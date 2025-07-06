#include <stdint.h>
#include <stdio.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_i2c.h"

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
  int a_x;   // Accelerometer x
  int a_y;   // Accelerometer y
  int a_z;   // Accelerometer z
  int g_rx;  // Gyroscope rx
  int g_ry;  // Gyroscope ry
  int g_rz;  // Gyroscope rz
  int temp;  // Temperature
} MPU6050Data;

MPU6050Data convert_gyro_data(uint8_t *buf);
void i2c_driver_setup();

int main(void) {
  i2c_driver_setup();

  uint8_t gyro_addr = 0x68;
  uint8_t wake_mpu[] = {0x6B, 0x00};

  printf("Starting...\n\n");

  i2c_master_send(I2C_PORT, wake_mpu, 2, gyro_addr, I2C_STOP);

  uint8_t tx_byte = 0x3B;
  uint8_t rx_buff[14] = {0x0};

  for (;;) {
    i2c_master_send(I2C_PORT, &tx_byte, 1, gyro_addr, I2C_NO_STOP);
    i2c_master_receive(I2C_PORT, rx_buff, 14, gyro_addr);

    MPU6050Data mpu_data = convert_gyro_data(rx_buff);

    printf("Accel x: %d\n", mpu_data.a_x);
    printf("Accel y: %d\n", mpu_data.a_y);
    printf("Accel z: %d\n", mpu_data.a_z);
    printf("Gyro rx: %d\n", mpu_data.g_rx);
    printf("Gyro ry: %d\n", mpu_data.g_ry);
    printf("Gyro rz: %d\n", mpu_data.g_rz);
    printf("MPU temperature: %d\n\n", mpu_data.temp);

    WAIT(SLOW);
  }
}

MPU6050Data convert_gyro_data(uint8_t *buf) {
  MPU6050Data ret;

  ret.a_x = (buf[0] << 8) + buf[1];
  ret.a_y = (buf[2] << 8) + buf[3];
  ret.a_z = (buf[4] << 8) + buf[5];
  ret.temp = (int)(((float)(buf[6] << 8) + buf[7]) / 340 + 36);
  ret.g_rx = (buf[8] << 8) + buf[9];
  ret.g_ry = (buf[10] << 8) + buf[11];
  ret.g_rz = (buf[12] << 8) + buf[13];

  return ret;
}

void i2c_driver_setup() {
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
                                    .dma_enable = I2C_DISABLE,
                                    .enable_on_init = I2C_ENABLE}};
  i2c_init(&i2c_handle);
}
