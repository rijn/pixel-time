#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "adxl345.h"

static const char *TAG = "adxl345";

ESP_EVENT_DEFINE_BASE(SENSOR_EVENT);

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID 0x00 // Device ID
#define ADXL345_RESERVED1 0x01 // Reserved. Do Not Access.
#define ADXL345_THRESH_TAP 0x1D // Tap Threshold.
#define ADXL345_OFSX 0x1E // X-Axis Offset.
#define ADXL345_OFSY 0x1F // Y-Axis Offset.
#define ADXL345_OFSZ 0x20 // Z- Axis Offset.
#define ADXL345_DUR 0x21 // Tap Duration.
#define ADXL345_LATENT 0x22 // Tap Latency.
#define ADXL345_WINDOW 0x23 // Tap Window.
#define ADXL345_THRESH_ACT 0x24 // Activity Threshold
#define ADXL345_THRESH_INACT 0x25 // Inactivity Threshold
#define ADXL345_TIME_INACT 0x26 // Inactivity Time
#define ADXL345_ACT_INACT_CTL                                                  \
  0x27 // Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF 0x28 // Free-Fall Threshold.
#define ADXL345_TIME_FF 0x29 // Free-Fall Time.
#define ADXL345_TAP_AXES 0x2A // Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS 0x2B // Source of Tap/Double Tap
#define ADXL345_BW_RATE 0x2C // Data Rate and Power mode Control
#define ADXL345_POWER_CTL 0x2D // Power-Saving Features Control
#define ADXL345_INT_ENABLE 0x2E // Interrupt Enable Control
#define ADXL345_INT_MAP 0x2F // Interrupt Mapping Control
#define ADXL345_INT_SOURCE 0x30 // Source of Interrupts
#define ADXL345_DATA_FORMAT 0x31 // Data Format Control
#define ADXL345_DATAX0 0x32 // X-Axis Data 0
#define ADXL345_DATAX1 0x33 // X-Axis Data 1
#define ADXL345_DATAY0 0x34 // Y-Axis Data 0
#define ADXL345_DATAY1 0x35 // Y-Axis Data 1
#define ADXL345_DATAZ0 0x36 // Z-Axis Data 0
#define ADXL345_DATAZ1 0x37 // Z-Axis Data 1
#define ADXL345_FIFO_CTL 0x38 // FIFO Control
#define ADXL345_FIFO_STATUS 0x39 // FIFO Status

#define ADXL345_BW_1600 0xF // 1111		IDD = 40uA
#define ADXL345_BW_800 0xE // 1110		IDD = 90uA
#define ADXL345_BW_400 0xD // 1101		IDD = 140uA
#define ADXL345_BW_200 0xC // 1100		IDD = 140uA
#define ADXL345_BW_100 0xB // 1011		IDD = 140uA
#define ADXL345_BW_50 0xA // 1010		IDD = 140uA
#define ADXL345_BW_25 0x9 // 1001		IDD = 90uA
#define ADXL345_BW_12_5 0x8 // 1000		IDD = 60uA
#define ADXL345_BW_6_25 0x7 // 0111		IDD = 50uA
#define ADXL345_BW_3_13 0x6 // 0110		IDD = 45uA
#define ADXL345_BW_1_56 0x5 // 0101		IDD = 40uA
#define ADXL345_BW_0_78 0x4 // 0100		IDD = 34uA
#define ADXL345_BW_0_39 0x3 // 0011		IDD = 23uA
#define ADXL345_BW_0_20 0x2 // 0010		IDD = 23uA
#define ADXL345_BW_0_10 0x1 // 0001		IDD = 23uA
#define ADXL345_BW_0_05 0x0 // 0000		IDD = 23uA

/************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN 0x00 // INT1: 0
#define ADXL345_INT2_PIN 0x01 // INT2: 1

/********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT 0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT 0x02
#define ADXL345_INT_WATERMARK_BIT 0x01
#define ADXL345_INT_OVERRUNY_BIT 0x00

#define ADXL345_DATA_READY 0x07
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05
#define ADXL345_ACTIVITY 0x04
#define ADXL345_INACTIVITY 0x03
#define ADXL345_FREE_FALL 0x02
#define ADXL345_WATERMARK 0x01
#define ADXL345_OVERRUNY 0x00

/****************************** ERRORS ******************************/
#define ADXL345_OK 1 // No Error
#define ADXL345_ERROR 0 // Error Exists

#define ADXL345_NO_ERROR 0 // Initial State
#define ADXL345_READ_ERROR 1 // Accelerometer Reading Error
#define ADXL345_BAD_ARG 2 // Bad Argument

#define ADXL345_SENSOR_ADDR 0x1D

double gains[3];

void i2c_master_adxl345_set_range(int val) {
  uint8_t _s;
  uint8_t _b;

  switch (val) {
  case 2:
    _s = 0b00000000;
    break;
  case 4:
    _s = 0b00000001;
    break;
  case 8:
    _s = 0b00000010;
    break;
  case 16:
    _s = 0b00000011;
    break;
  default:
    _s = 0b00000000;
  }
  i2c_master_read_slave_reg(ADXL345_SENSOR_ADDR,
                            ADXL345_DATA_FORMAT, &_b, sizeof(uint8_t));
  _s |= (_b & 0b11101100);
  i2c_master_write_slave_reg(ADXL345_SENSOR_ADDR,
                             ADXL345_DATA_FORMAT, &_s, sizeof(uint8_t));
}

void i2c_master_adxl345_begin(void) {
  gains[0] = 0.00376390; // Original gain 0.00376390
  gains[1] = 0.00376009; // Original gain 0.00376009
  gains[2] = 0.00349265; // Original gain 0.00349265

  // uint8_t chipid;
  // i2c_master_read_slave_reg(I2C_MASTER_NUM, BMP280_SENSOR_ADDR,
  //                           BMP280_REGISTER_CHIPID, &chipid,
  //                           sizeof(uint8_t));
  // ESP_LOGI(TAG, "Chip ID = 0x%02x", chipid);

  uint8_t power_config_value = 0b00011000; // 00011000d
  i2c_master_write_slave_reg(ADXL345_SENSOR_ADDR,
                             ADXL345_POWER_CTL, &power_config_value,
                             sizeof(uint8_t));

  i2c_master_adxl345_set_range(16);
}

void i2c_master_adxl345_read_acceleration(int *x, int *y, int *z) {
  uint8_t _buffer[6];
  i2c_master_read_slave_reg(ADXL345_SENSOR_ADDR, ADXL345_DATAX0,
                            _buffer, sizeof(_buffer));

  // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
  *x = (int16_t)((((int)_buffer[1]) << 8) | _buffer[0]);
  *y = (int16_t)((((int)_buffer[3]) << 8) | _buffer[2]);
  *z = (int16_t)((((int)_buffer[5]) << 8) | _buffer[4]);
}

void i2c_master_adxl345_read_acceleration_g(double *xyz) {
  int xyz_int[3];
  i2c_master_adxl345_read_acceleration(xyz_int, xyz_int + 1, xyz_int + 2);
  for (uint8_t i = 0; i < 3; i++) {
    xyz[i] = xyz_int[i] * gains[i];
  }
}

void check_acceperation(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100 / portTICK_RATE_MS;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    double acceleration[3];
    i2c_master_adxl345_read_acceleration_g(acceleration);

    ESP_LOGI(TAG, "Acceleration [%6.4lf, %6.4lf, %6.4lf]", acceleration[0],
             acceleration[1], acceleration[2]);

    // esp_event_post(SENSOR_EVENT, SENSOR_EVENT_TEMPERATURE_UPDATED,
    // &temperature,
    //                sizeof(temperature), portMAX_DELAY);
  }
}

void initialize_adxl345(void) {
  i2c_master_adxl345_begin();

  xTaskCreate(check_acceperation, "check_acceperation", 4096, NULL, 1, NULL);
}
