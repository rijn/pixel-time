#include "driver/i2c.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include <pthread.h>
#include <stdio.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

static const char *TAG = "i2c";

static gpio_num_t i2c_gpio_sda = 23;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 400000;
static i2c_port_t i2c_port = I2C_NUM_0;

static esp_err_t i2c_master_driver_initialize(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = i2c_gpio_sda,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = i2c_gpio_scl,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = i2c_frequency,
      // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_*
      // flags to choose i2c source clock here. */
  };
  return i2c_param_config(i2c_port, &conf);
}

void i2c_detect(void) {
  uint8_t address;
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  for (int i = 0; i < 128; i += 16) {
    printf("%02x: ", i);
    for (int j = 0; j < 16; j++) {
      fflush(stdout);
      address = i + j;
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_stop(cmd);
      esp_err_t ret =
          i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd);
      if (ret == ESP_OK) {
        printf("%02x ", address);
      } else if (ret == ESP_ERR_TIMEOUT) {
        printf("UU ");
      } else {
        printf("-- ");
      }
    }
    printf("\r\n");
  }
  return;
}

pthread_mutex_t i2c_lock;

esp_err_t i2c_master_init(void) {
  i2c_master_driver_initialize();
  if (pthread_mutex_init(&i2c_lock, NULL) != 0) {
    printf("\n mutex init has failed\n");
    return 1;
  }
  return i2c_driver_install(i2c_port, I2C_MODE_MASTER,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack |
 * read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_master_read_slave_reg(uint8_t i2c_addr, uint8_t i2c_reg,
                                    uint8_t *data_rd, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }

  pthread_mutex_lock(&i2c_lock);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_addr << 1), ACK_CHECK_EN);
  i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_addr << 1) | READ_BIT, ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  pthread_mutex_unlock(&i2c_lock);

  return ret;
}

/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  |
 * stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
esp_err_t i2c_master_write_slave_reg(uint8_t i2c_addr, uint8_t i2c_reg,
                                     uint8_t *data_wr, size_t size) {
  pthread_mutex_lock(&i2c_lock);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
  i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  pthread_mutex_unlock(&i2c_lock);

  return ret;
}

// For VEML6030 reg, process LSB first.
uint16_t i2c_master_read_slave_reg_16(uint8_t i2c_addr, uint8_t i2c_reg) {
  uint8_t data[2];
  i2c_master_read_slave_reg(i2c_addr, i2c_reg, data, 2);
  return (data[1] << 8) | data[0];
}

esp_err_t i2c_master_write_slave_reg_16_with_mask(uint8_t i2c_addr,
                                                  uint8_t i2c_reg,
                                                  uint16_t _mask,
                                                  uint16_t _bits,
                                                  uint8_t _start_position) {
  uint16_t _bits_to_write;

  _bits_to_write = i2c_master_read_slave_reg_16(i2c_addr, i2c_reg);
  _bits_to_write &= _mask;
  _bits_to_write |= (_bits << _start_position);

  _bits_to_write = (_bits_to_write & 0xFF00) >> 8 | _bits_to_write << 8;

  return i2c_master_write_slave_reg(i2c_addr, i2c_reg,
                                    (uint8_t *)&_bits_to_write, 2);
}