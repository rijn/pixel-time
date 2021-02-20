#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp32_i2s_parallel_v2.h"

#define ESP32_I2S_DMA_MODE I2S_PARALLEL_WIDTH_8
#define ESP32_I2S_DMA_STORAGE_TYPE uint8_t
#define ESP32_I2S_CLOCK_SPEED (1000000UL) // @ 1Mhz
#define CLKS_DURING_LATCH 0               // Not used.

#define DATA_PIN 16
#define SCLK_PIN 17
#define XLAT_PIN 19
#define BLANK_PIN 18
#define ADDR1_PIN 13
#define ADDR2_PIN 21
#define ADDR3_PIN 15

#define BIT_DATA (1 << 0)
#define BIT_SCLK (1 << 1)
#define BIT_XLAT (1 << 2)
#define BIT_BLANK (1 << 3)
#define BIT_ADDR1 (1 << 4)
#define BIT_ADDR2 (1 << 5)
#define BIT_ADDR3 (1 << 6)

#define PIXEL_COLOR_DEPTH_BITS 12
#define COLOR_CHANNELS_PER_PIXEL 3
#define PIXELS_PER_ROW 8
#define ROWS_PER_FRAME 8

#define XLAT_BITS 12

typedef struct {
  ESP32_I2S_DMA_STORAGE_TYPE bits[PIXEL_COLOR_DEPTH_BITS * 2];
} display_pixel_t;

typedef struct {
  display_pixel_t pixels[PIXELS_PER_ROW];
} display_channel_t;

typedef struct {
  display_channel_t channels[COLOR_CHANNELS_PER_PIXEL];
  ESP32_I2S_DMA_STORAGE_TYPE xlat_bits[XLAT_BITS];
} display_row_t;

typedef struct {
  display_row_t rows[ROWS_PER_FRAME];
} display_frame_t;

bool initialize_display(void);

#endif
