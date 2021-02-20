#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "i2c.h"
#include "veml6030.h"

#define ENABLE 0x01
#define DISABLE 0x00
#define SHUTDOWN 0x01
#define POWER 0x00
#define NO_INT 0x00
#define INT_HIGH 0x01
#define INT_LOW 0x02
#define UNKNOWN_ERROR 0xFF

// 7-Bit address options
const uint8_t defAddr = 0x48;
const uint8_t altAddr = 0x10;

enum VEML6030_16BIT_REGISTERS {

  SETTING_REG = 0x00,
  H_THRESH_REG,
  L_THRESH_REG,
  POWER_SAVE_REG,
  AMBIENT_LIGHT_DATA_REG,
  WHITE_LIGHT_DATA_REG,
  INTERRUPT_REG

};

enum VEML6030_16BIT_REG_MASKS {

  THRESH_MASK = 0x0,
  GAIN_MASK = 0xE7FF,
  INTEG_MASK = 0xFC3F,
  PERS_PROT_MASK = 0xFFCF,
  INT_EN_MASK = 0xFFFD,
  SD_MASK = 0xFFFE,
  POW_SAVE_EN_MASK = 0x06, // Most of this register is reserved
  POW_SAVE_MASK = 0x01,    // Most of this register is reserved
  INT_MASK = 0xC000

};

enum REGISTER_BIT_POSITIONS {

  NO_SHIFT = 0x00,
  INT_EN_POS = 0x01,
  PSM_POS = 0x01,
  PERS_PROT_POS = 0x04,
  INTEG_POS = 0x06,
  GAIN_POS = 0xB,
  INT_POS = 0xE

};

// Table of lux conversion values depending on the integration time and gain.
// The arrays represent the all possible integration times and the index of the
// arrays represent the register's gain settings, which is directly analgous to
// their bit representations.
const float eightHIt[] = {.0036, .0072, .0288, .0576};
const float fourHIt[] = {.0072, .0144, .0576, .1152};
const float twoHIt[] = {.0144, .0288, .1152, .2304};
const float oneHIt[] = {.0288, .0576, .2304, .4608};
const float fiftyIt[] = {.0576, .1152, .4608, .9216};
const float twentyFiveIt[] = {.1152, .2304, .9216, 1.8432};

static const char *TAG = "veml6030";

#define VEML6030_SENSOR_ADDR 0x48

void delay(int number_of_seconds) {
  // Converting time into milli_seconds
  int milli_seconds = 1000 * number_of_seconds;

  // Storing start time
  clock_t start_time = clock();

  // looping till required time is not achieved
  while (clock() < start_time + milli_seconds);
}

void i2c_master_veml6030_power_on(void) {
  i2c_master_write_slave_reg_16_with_mask(VEML6030_SENSOR_ADDR, SETTING_REG,
                                          SD_MASK, POWER, NO_SHIFT);
  delay(4);
}

void i2c_master_veml6030_set_gain(float gainVal) {
  uint16_t bits;

  if (gainVal == 1.00)
    bits = 0;
  else if (gainVal == 2.00)
    bits = 1;
  else if (gainVal == .125)
    bits = 2;
  else if (gainVal == .25)
    bits = 3;
  else
    return;

  i2c_master_write_slave_reg_16_with_mask(VEML6030_SENSOR_ADDR, SETTING_REG,
                                          GAIN_MASK, bits, GAIN_POS);
}

float i2c_master_veml6030_read_gain() {
  uint16_t regVal = i2c_master_read_slave_reg_16(VEML6030_SENSOR_ADDR,
                                                 SETTING_REG); // Get register
  regVal &= (~GAIN_MASK);        // Invert the gain mask to _keep_ the gain
  regVal = (regVal >> GAIN_POS); // Move values to front of the line.

  if (regVal == 0)
    return 1;
  else if (regVal == 1)
    return 2;
  else if (regVal == 2)
    return .125;
  else if (regVal == 3)
    return .25;
  else
    return UNKNOWN_ERROR;
}

void i2c_master_veml6030_set_integ_time(uint16_t time) {
  uint16_t bits;

  if (time == 100) // Default setting.
    bits = 0;
  else if (time == 200)
    bits = 1;
  else if (time == 400)
    bits = 2;
  else if (time == 800)
    bits = 3;
  else if (time == 50)
    bits = 8;
  else if (time == 25)
    bits = 12;
  else
    return;

  i2c_master_write_slave_reg_16_with_mask(VEML6030_SENSOR_ADDR, SETTING_REG,
                                          INTEG_MASK, bits, INTEG_POS);
}

uint16_t i2c_master_veml6030_read_integ_time() {
  uint16_t regVal =
      i2c_master_read_slave_reg_16(VEML6030_SENSOR_ADDR, SETTING_REG);
  regVal &= (~INTEG_MASK);
  regVal = (regVal >> INTEG_POS);

  if (regVal == 0)
    return 100;
  else if (regVal == 1)
    return 200;
  else if (regVal == 2)
    return 400;
  else if (regVal == 3)
    return 800;
  else if (regVal == 8)
    return 50;
  else if (regVal == 12)
    return 25;
  else
    return UNKNOWN_ERROR;
}

uint32_t i2c_master_veml6030_calculate_lux(uint16_t _lightBits) {
  float _luxConv;
  uint8_t _convPos;

  float _gain = i2c_master_veml6030_read_gain();
  uint16_t _integTime = i2c_master_veml6030_read_integ_time();

  // Here the gain is checked to get the position of the conversion value
  // within the integration time arrays. These values also represent the bit
  // values for setting the gain.
  if (_gain == 2.00)
    _convPos = 0;
  else if (_gain == 1.00)
    _convPos = 1;
  else if (_gain == .25)
    _convPos = 2;
  else if (_gain == .125)
    _convPos = 3;
  else
    return UNKNOWN_ERROR;

  // Here we check the integration time which determines which array we probe
  // at the position determined above.
  if (_integTime == 800)
    _luxConv = eightHIt[_convPos];
  else if (_integTime == 400)
    _luxConv = fourHIt[_convPos];
  else if (_integTime == 200)
    _luxConv = twoHIt[_convPos];
  else if (_integTime == 100)
    _luxConv = oneHIt[_convPos];
  else if (_integTime == 50)
    _luxConv = fiftyIt[_convPos];
  else if (_integTime == 25)
    _luxConv = twentyFiveIt[_convPos];
  else
    return UNKNOWN_ERROR;

  // Multiply the value from the 16 bit register to the conversion value and
  // return it.
  uint32_t _calculatedLux = (_luxConv * _lightBits);
  return _calculatedLux;
}

static uint32_t lux_compensation(uint32_t _luxVal) {
  // Polynomial is pulled from pg 10 of the datasheet.
  uint32_t _compLux = (.00000000000060135 * (pow(_luxVal, 4))) -
                      (.0000000093924 * (pow(_luxVal, 3))) +
                      (.000081488 * (pow(_luxVal, 2))) + (1.0023 * _luxVal);
  return _compLux;
}

uint32_t i2c_master_veml6030_read_light() {

  uint16_t lightBits = i2c_master_read_slave_reg_16(VEML6030_SENSOR_ADDR,
                                                    AMBIENT_LIGHT_DATA_REG);
  uint32_t luxVal = i2c_master_veml6030_calculate_lux(lightBits);

  if (luxVal > 1000) {
    uint32_t compLux = lux_compensation(luxVal);
    return compLux;
  } else
    return luxVal;
}

void check_ambient_light(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100 / portTICK_RATE_MS;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint32_t lux = i2c_master_veml6030_read_light();

    ESP_LOGI(TAG, "Ambient light [%d] lux", lux);

    // esp_event_post(SENSOR_EVENT, SENSOR_EVENT_TEMPERATURE_UPDATED,
    // &temperature,
    //                sizeof(temperature), portMAX_DELAY);
  }
}

void initialize_veml6030(void) {
  i2c_master_veml6030_power_on();

  // Possible values: .125, .25, 1, 2
  // Both .125 and .25 should be used in most cases except darker rooms.
  // A gain of 2 should only be used if the sensor will be covered by a dark
  // glass.
  i2c_master_veml6030_set_gain(0.125);
  // Possible integration times in milliseconds: 800, 400, 200, 100, 50, 25
  // Higher times give higher resolutions and should be used in darker light.
  i2c_master_veml6030_set_integ_time(100);

  xTaskCreate(check_ambient_light, "check_ambient_light", 4096, NULL, 1, NULL);
}
