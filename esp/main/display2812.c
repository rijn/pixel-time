#include "driver/rmt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdlib.h>
#include <string.h>

#include "esp32_i2s_parallel.h"
#include "esp_heap_caps.h"

#include "gfxfont.h"
#include "Picopixel.h"
#include "TomThumb.h"

#include "display2812.h"

static const char *TAG = "display2812";

#define LED_RMT_TX_CHANNEL 0
#define LED_RMT_TX_GPIO 25

#define WIDTH 18
#define HEIGHT 6

#define NUM_LEDS ((WIDTH * HEIGHT))
#define BITS_PER_LED_CMD 24
#define LED_BUFFER_ITEMS ((NUM_LEDS * BITS_PER_LED_CMD))

// These values are determined by measuring pulse timing with logic analyzer and
// adjusting to match datasheet.
#define T0H 14 // 0 bit high time
#define T1H 52 // 1 bit high time
#define TL 52 // low time for either bi

// This structure is used for indicating what the colors of each LED should be
// set to.
typedef struct {
  uint32_t leds[NUM_LEDS];
} matrix_state_t;

matrix_state_t frame_0;

// This is the buffer which the hw peripheral will access while pulsing the
// output pin
rmt_item32_t led_data_buffer[LED_BUFFER_ITEMS];

void setup_rmt_data_buffer(matrix_state_t *frame);

void ws2812_control_init(void) {
  rmt_config_t config =
      RMT_DEFAULT_CONFIG_TX(LED_RMT_TX_GPIO, LED_RMT_TX_CHANNEL);
  // set counter clock to 40MHz
  config.clk_div = 2;
  // rmt_config_t config;
  // config.rmt_mode = RMT_MODE_TX;
  // config.channel = LED_RMT_TX_CHANNEL;
  // config.gpio_num = LED_RMT_TX_GPIO;
  // config.mem_block_num = 3;
  // config.tx_config.loop_en = false;
  // config.tx_config.carrier_en = false;
  // config.tx_config.idle_output_en = true;
  // config.tx_config.idle_level = 0;
  // config.clk_div = 2;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

void ws2812_write_frame(matrix_state_t *frame) {
  setup_rmt_data_buffer(frame);
  ESP_ERROR_CHECK(rmt_write_items(LED_RMT_TX_CHANNEL, led_data_buffer,
                                  LED_BUFFER_ITEMS, false));
  ESP_ERROR_CHECK(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY));
}

void setup_rmt_data_buffer(matrix_state_t *frame) {
  for (uint32_t led = 0; led < NUM_LEDS; led++) {
    uint32_t bits_to_send = frame->leds[led];
    uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);
    for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
      uint32_t bit_is_set = bits_to_send & mask;
      led_data_buffer[led * BITS_PER_LED_CMD + bit] =
          bit_is_set ? (rmt_item32_t){{{T1H, 1, TL, 0}}}
                     : (rmt_item32_t){{{T0H, 1, TL, 0}}};
      mask >>= 1;
    }
  }
}

void clear_frame(matrix_state_t *frame) {
  memset(frame, 0, sizeof(matrix_state_t));
}

void draw_pixel(matrix_state_t *frame, uint8_t x, uint8_t y, uint32_t color) {
  uint8_t id = (HEIGHT - 1 - y) * WIDTH + (y % 2 == 0 ? WIDTH - 1 - x : x);
  frame->leds[id] = color;
}

void draw_rect(matrix_state_t *frame, int16_t x, int16_t y, int16_t w,
               int16_t h, uint32_t color) {
  for (int16_t i = x; i < x + w; i++) {
    for (int16_t j = y; j < y + h; j++) {
      draw_pixel(frame, i, j, color);
    }
  }
}

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
  return gfxFont->bitmap;
}

GFXfont *gfxFont = (GFXfont *)&TomThumb;

void draw_char(matrix_state_t *frame, int16_t x, int16_t y, unsigned char c,
               uint32_t color, uint8_t size_x, uint8_t size_y) {
  c -= (uint8_t)pgm_read_byte(&gfxFont->first);
  GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
  uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

  uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
  uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
  int8_t xo = pgm_read_byte(&glyph->xOffset),
         yo = pgm_read_byte(&glyph->yOffset);
  uint8_t xx, yy, bits = 0, bit = 0;
  int16_t xo16 = 0, yo16 = 0;

  if (size_x > 1 || size_y > 1) {
    xo16 = xo;
    yo16 = yo;
  }

  for (yy = 0; yy < h; yy++) {
    for (xx = 0; xx < w; xx++) {
      if (!(bit++ & 7)) {
        bits = pgm_read_byte(&bitmap[bo++]);
      }
      if (bits & 0x80) {
        if (size_x == 1 && size_y == 1) {
          draw_pixel(frame, x + xo + xx, y + yo + yy, color);
        } else {
          draw_rect(frame, x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y,
                    size_x, size_y, color);
        }
      }
      bits <<= 1;
    }
  }
}

static EventGroupHandle_t s_display_event_group;

void display_test_task() {
  for (;;)
    for (uint8_t x = 0; x < WIDTH; x++) {
      for (uint8_t y = 0; y < HEIGHT; y++) {
        clear_frame(&frame_0);
        draw_pixel(&frame_0, x, y, 0xFFFFFF);
        ws2812_write_frame(&frame_0);
      }
    }
}

bool initialize_display_2812(void) {
  ESP_LOGI(TAG, "Initialize 2812 matrix.");
  ws2812_control_init();

  clear_frame(&frame_0);
  ws2812_write_frame(&frame_0);

  // xTaskCreatePinnedToCore(display_test_task, "display_test_task", 4096, NULL,
  // 3,
  //                         NULL, 1);

  draw_char(&frame_0, 0, 5, '1', 0x2F2F2F, 1, 1);
  draw_char(&frame_0, 4, 5, '2', 0x2F2F2F, 1, 1);
  draw_char(&frame_0, 8, 5, '4', 0x2F2F2F, 1, 1);
  draw_char(&frame_0, 12, 5, '5', 0x2F2F2F, 1, 1);
  ws2812_write_frame(&frame_0);

  return true;
}
