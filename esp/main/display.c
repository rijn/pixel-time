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

#include "esp32_i2s_parallel_v2.h"
#include "esp_heap_caps.h"

#include "display.h"

static const char *TAG = "display";

/* Pixel data is organized from LSB to MSB sequentially by row, from row 0 to
 * row matrixHeight/matrixRowsInParallel (two rows of pixels are refreshed in
 * parallel) Memory is allocated (malloc'd) by the row, and not in one massive
 * chunk, for flexibility.
 */
display_row_t *matrix_row_framebuffer_malloc[ROWS_PER_FRAME];

// ESP 32 DMA Linked List descriptor
int desccount = 0;
lldesc_t *dmadesc_a = {0};
lldesc_t *dmadesc_b = {0};

int lsb_msb_transition_bit = 0;

bool double_buffering_enabled = false;

bool allocate_dma_memory(void) {
  int _num_frame_buffers = (double_buffering_enabled) ? 2 : 1;
  size_t _frame_buffer_memory_required =
      sizeof(display_frame_t) * _num_frame_buffers;
  size_t _dma_linked_list_memory_required = 0;
  size_t _total_dma_capable_memory_reserved = 0;

  // 1. Calculate the amount of DMA capable memory that's actually available
  ESP_LOGI(TAG, "DMA memory blocks available before any malloc's:");
  heap_caps_print_heap_info(MALLOC_CAP_DMA);
  ESP_LOGI(TAG,
           "We're going to need %d bytes of SRAM just for the frame buffer(s).",
           _frame_buffer_memory_required);
  ESP_LOGI(TAG, "The total amount of DMA capable SRAM memory is %d bytes.",
           heap_caps_get_free_size(MALLOC_CAP_DMA));
  ESP_LOGI(TAG, "Largest DMA capable SRAM memory block is %d bytes.",
           heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  if (heap_caps_get_free_size(MALLOC_CAP_DMA) < _frame_buffer_memory_required) {
    ESP_LOGE(TAG,
             "######### Insufficient memory for requested resolution. Reduce "
             "MATRIX_COLOR_DEPTH and try again.\tAdditional %d bytes of "
             "memory required.",
             (_frame_buffer_memory_required -
              heap_caps_get_free_size(MALLOC_CAP_DMA)));
    return false;
  }

  // Alright, theoretically we should be OK, so let us do this, so
  // lets allocate a chunk of memory for each row (a row could span multiple
  // panels if chaining is in place)
  for (int malloc_num = 0; malloc_num < ROWS_PER_FRAME; malloc_num++) {
    matrix_row_framebuffer_malloc[malloc_num] =
        (display_row_t *)heap_caps_malloc(
            (sizeof(display_row_t) * _num_frame_buffers), MALLOC_CAP_DMA);
    // If the ESP crashes here, then we must have a horribly fragmented memory
    // space, or trying to allocate a ludicrous resolution.
    ESP_LOGI(TAG,
             "Malloc'ing %d bytes of memory @ address %p for frame row %d.",
             (sizeof(display_row_t) * _num_frame_buffers),
             matrix_row_framebuffer_malloc[malloc_num], malloc_num);
    if (matrix_row_framebuffer_malloc[malloc_num] == NULL) {
      ESP_LOGE(TAG,
               "ERROR: Couldn't malloc matrix_row_framebuffer %d! "
               "Critical fail.",
               malloc_num);
      return false;
    }
  }

  _total_dma_capable_memory_reserved += _frame_buffer_memory_required;

  int num_dma_descriptors_per_row = 1;

  _dma_linked_list_memory_required = num_dma_descriptors_per_row *
                                     ROWS_PER_FRAME * _num_frame_buffers *
                                     sizeof(lldesc_t);
  ESP_LOGI(TAG,
           "Descriptors for lsb_msb_transition_bit of %d/%d with %d frame "
           "rows require "
           "%d bytes of DMA RAM with %d num_dma_descriptors_per_row.",
           lsb_msb_transition_bit, PIXEL_COLOR_DEPTH_BITS - 1, ROWS_PER_FRAME,
           _dma_linked_list_memory_required, num_dma_descriptors_per_row);

  _total_dma_capable_memory_reserved += _dma_linked_list_memory_required;

  // Do a final check to see if we have enough space for the additional DMA
  // linked list descriptors that will be required to link it all up!
  if (_dma_linked_list_memory_required >
      heap_caps_get_largest_free_block(MALLOC_CAP_DMA)) {
    ESP_LOGE(TAG, "ERROR: Not enough SRAM left over for DMA linked-list "
                  "descriptor memory reservation! Oh so close!");
    return false;
  }

  // malloc the DMA linked list descriptors that i2s_parallel will need
  desccount = num_dma_descriptors_per_row * ROWS_PER_FRAME;

  // lldesc_t * dmadesc_a = (lldesc_t *)heap_caps_malloc(desccount *
  // sizeof(lldesc_t), MALLOC_CAP_DMA);
  dmadesc_a = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t),
                                           MALLOC_CAP_DMA);
  ESP_LOGI(TAG, "desccount = %d", desccount);
  assert("Can't allocate descriptor framebuffer a");
  if (!dmadesc_a) {
    ESP_LOGE(TAG, "ERROR: Could not malloc descriptor framebuffer a.");
    return false;
  }

  ESP_LOGI(TAG, "*** ESP32-HUB75-MatrixPanel-I2S-DMA: Memory Allocations "
                "Complete *** ");
  ESP_LOGI(TAG, "Total memory that was reserved: %d kB.",
           _total_dma_capable_memory_reserved / 1024);
  ESP_LOGI(TAG, "... of which was used for the DMA Linked List(s): %d kB.",
           _dma_linked_list_memory_required / 1024);

  ESP_LOGI(TAG,
           "Heap Memory Available: %d bytes total. Largest free block: %d "
           "bytes.",
           heap_caps_get_free_size(0), heap_caps_get_largest_free_block(0));
  ESP_LOGI(TAG,
           "General RAM Available: %d bytes total. Largest free block: %d "
           "bytes.",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

  ESP_LOGI(TAG, "DMA capable memory map available after malloc's: ");
  heap_caps_print_heap_info(MALLOC_CAP_DMA);

  return true;
}

bool configure_dma(void) {
  ESP_LOGI(TAG, "configureDMA(): Starting configuration of DMA engine.");

  lldesc_t *previous_dmadesc_a = 0;
  lldesc_t *previous_dmadesc_b = 0;
  int current_dmadescriptor_offset = 0;

  // HACK: If we need to split the payload in 1/2 so that it doesn't breach
  // DMA_MAX, lets do it by the color_depth.
  int num_dma_payload_color_depths = PIXEL_COLOR_DEPTH_BITS;
  // if (sizeof(display_row_color_depth_t) > DMA_MAX) {
  //   num_dma_payload_color_depths = 1;
  // }

  // Fill DMA linked lists for both frames (as in, halves of the HUB75 panel)
  // and if double buffering is enabled, link it up for both buffers.
  for (int row = 0; row < ROWS_PER_FRAME; row++) {

    // Split framebuffer malloc hack 'improvement'
    display_row_t *fb_malloc_ptr = matrix_row_framebuffer_malloc[row];

    ESP_LOGI(TAG, "DMA payload of %d bytes. DMA_MAX is %d. Addr %p",
             sizeof(display_row_t), DMA_MAX, fb_malloc_ptr);

    // first set of data is LSB through MSB, single pass (IF TOTAL SIZE <
    // DMA_MAX) - all color bits are displayed once, which takes care of
    // everything below and inlcluding LSBMSB_TRANSITION_BIT NOTE: size must be
    // less than DMA_MAX - worst case for library: 16-bpp with 256 pixels per
    // row would exceed this, need to break into two
    link_dma_desc(&dmadesc_a[current_dmadescriptor_offset], previous_dmadesc_a,
                  fb_malloc_ptr, sizeof(display_row_t));
    previous_dmadesc_a = &dmadesc_a[current_dmadescriptor_offset];

    // if (double_buffering_enabled) {
    //   link_dma_desc(&dmadesc_b[current_dmadescriptor_offset],
    //                 previous_dmadesc_b, &(fb_malloc_ptr[1].pixels[0].data),
    //                 sizeof(display_row_t));
    //   previous_dmadesc_b = &dmadesc_b[current_dmadescriptor_offset];
    // }

    current_dmadescriptor_offset++;

    // If the number of pixels per row is to great for the size of a DMA
    // payload, so we need to split what we were going to send above.
    if (sizeof(display_row_t) > DMA_MAX) {

      ESP_LOGI(
          TAG,
          "Spliting DMA payload for %d color depths into %d byte payloads.",
          PIXEL_COLOR_DEPTH_BITS - 1, sizeof(display_pixel_t));

      for (int cd = 1; cd < PIXEL_COLOR_DEPTH_BITS; cd++) {
        // first set of data is LSB through MSB, single pass - all color bits
        // are displayed once, which takes care of everything below and
        // inlcluding LSBMSB_TRANSITION_BIT
        // TODO: size must be less than DMA_MAX - worst case for library: 16 -
        // bpp with 256 pixels per row would exceed this, need to break into two
        link_dma_desc(&dmadesc_a[current_dmadescriptor_offset],
                      previous_dmadesc_a, fb_malloc_ptr,
                      sizeof(display_pixel_t));
        previous_dmadesc_a = &dmadesc_a[current_dmadescriptor_offset];

        if (double_buffering_enabled) {
          link_dma_desc(&dmadesc_b[current_dmadescriptor_offset],
                        previous_dmadesc_b, fb_malloc_ptr,
                        sizeof(display_pixel_t));
          previous_dmadesc_b = &dmadesc_b[current_dmadescriptor_offset];
        }

        current_dmadescriptor_offset++;

      } // additional linked list items
    }   // row depth struct

    // for (int i = lsb_msb_transition_bit + 1; i < PIXEL_COLOR_DEPTH_BITS; i++)
    // {
    // // binary time division setup: we need 2 of bit (LSBMSB_TRANSITION_BIT +
    // // 1) four of (LSBMSB_TRANSITION_BIT + 2), etc because we sweep through
    // to
    // // MSB each time, it divides the number of times we have to sweep in half
    // // (saving linked list RAM) we need 2^(i - LSBMSB_TRANSITION_BIT - 1) ==
    // 1
    // // << (i - LSBMSB_TRANSITION_BIT - 1) passes from i to MSB
    // // ESP_LOGI(TAG,"buffer %d: repeat %d times, size: %d, from %d - %d",
    // // current_dmadescriptor_offset, 1<<(i - lsb_msb_transition_bit - 1),
    // // (PIXEL_COLOR_DEPTH_BITS - i), i, PIXEL_COLOR_DEPTH_BITS-1);

    // ESP_LOGI(
    // TAG,
    // "configureDMA(): DMA Loops for PIXEL_COLOR_DEPTH_BITS %d is:
    // %d.", i, (1 << (i - lsb_msb_transition_bit - 1)));

    // for (int k = 0; k < (1 << (i - lsb_msb_transition_bit - 1)); k++) {
    // link_dma_desc(&dmadesc_a[current_dmadescriptor_offset],
    // previous_dmadesc_a, &(fb_malloc_ptr[0].pixels[i].data),
    // sizeof(display_pixel_t) * (PIXEL_COLOR_DEPTH_BITS - i));
    // previous_dmadesc_a = &dmadesc_a[current_dmadescriptor_offset];

    // if (double_buffering_enabled) {
    // link_dma_desc(&dmadesc_b[current_dmadescriptor_offset],
    // previous_dmadesc_b, &(fb_malloc_ptr[1].pixels[i].data),
    // sizeof(display_pixel_t) * (PIXEL_COLOR_DEPTH_BITS - i));
    // previous_dmadesc_b = &dmadesc_b[current_dmadescriptor_offset];
    // }

    // current_dmadescriptor_offset++;

    // } // end color depth ^ 2 linked list
    // }     // end color depth loop

  } // end frame rows

  ESP_LOGI(TAG,
           "configureDMA(): Configured LL structure. %d DMA Linked List "
           "descriptors populated.",
           current_dmadescriptor_offset);

  if (desccount != current_dmadescriptor_offset) {
    ESP_LOGI(TAG,
             "configureDMA(): ERROR! Expected descriptor count of %d != "
             "actual DMA descriptors of %d!",
             desccount, current_dmadescriptor_offset);
  }

  dmadesc_a[desccount - 1].eof = 1;
  dmadesc_a[desccount - 1].qe.stqe_next = (lldesc_t *)&dmadesc_a[0];

  // End markers for DMA LL
  if (double_buffering_enabled) {
    dmadesc_b[desccount - 1].eof = 1;
    dmadesc_b[desccount - 1].qe.stqe_next = (lldesc_t *)&dmadesc_b[0];
  } else {
    dmadesc_b = dmadesc_a; // link to same 'a' buffer
  }

  /*
      //End markers
      dmadesc_a[desccount-1].eof = 1;
      dmadesc_b[desccount-1].eof = 1;
      dmadesc_a[desccount-1].qe.stqe_next=(lldesc_t*)&dmadesc_a[0];
      dmadesc_b[desccount-1].qe.stqe_next=(lldesc_t*)&dmadesc_b[0];
  */
  // ESP_LOGI(TAG,"Performing I2S setup.\n");

  // i2s_parallel_config_t cfg = {
  //     .gpio_bus = {DATA_PIN, -1, XLAT_PIN, BLANK_PIN, ADDR1_PIN, ADDR2_PIN,
  //                  ADDR3_PIN, -1},
  //     .gpio_clk = SCLK_PIN,
  //     .clkspeed_hz =
  //         ESP32_I2S_CLOCK_SPEED,  // ESP32_I2S_CLOCK_SPEED,  // formula used
  //         is
  //                                 // 80000000L/(cfg->clkspeed_hz + 1), must
  //                                 // result in >=2.  Acceptable
  //                                 // values 26.67MHz, 20MHz,
  //                                 16MHz, 13.34MHz...
  //     .bits = ESP32_I2S_DMA_MODE, // ESP32_I2S_DMA_MODE,
  //     .bufa = 0,
  //     .bufb = 0,
  //     desccount,
  //     desccount,
  //     dmadesc_a,
  //     dmadesc_b};

  // // Setup I2S
  // i2s_parallel_setup_without_malloc(&I2S1, &cfg);

  i2s_parallel_config_t cfg = {.gpio_bus = {DATA_PIN, SCLK_PIN, XLAT_PIN, BLANK_PIN,
                                            ADDR1_PIN, ADDR2_PIN, ADDR3_PIN,
                                            -1},
                               .gpio_clk = -1,
                               .sample_rate = ESP32_I2S_CLOCK_SPEED,
                               .sample_width = ESP32_I2S_DMA_MODE,
                               .desccount_a = desccount,
                               .lldesc_a = dmadesc_a,
                               .desccount_b = desccount,
                               .lldesc_b = dmadesc_b};

  // Setup I2S
  i2s_parallel_driver_install(I2S_NUM_1, &cfg);

  // Start DMA Output
  i2s_parallel_send_dma(I2S_NUM_1, &dmadesc_a[0]);

  ESP_LOGI(TAG, "configureDMA(): DMA configuration completed on I2S1.");

  ESP_LOGI(TAG, "DMA Memory Map after DMA LL allocations: ");
  heap_caps_print_heap_info(MALLOC_CAP_DMA);

  return true;
}

void clear_buffer(void) {
  for (unsigned int matrix_frame_parallel_row = 0;
       matrix_frame_parallel_row < ROWS_PER_FRAME;
       matrix_frame_parallel_row++) {
    display_row_t *fb_row_malloc_ptr = (display_row_t *)
        matrix_row_framebuffer_malloc[matrix_frame_parallel_row];
    memset(fb_row_malloc_ptr, 0, sizeof(display_row_t));
  }
}

void set_address_and_latch(void) {
  // TODO: set address pin

  // set latch
}

const uint8_t heart_map[] = {
    /*Pixel format: Blue: 8 bit, Green: 8 bit, Red: 8 bit, Fix 0xFF: 8 bit, */
    0x09, 0x04, 0x13, 0xff, 0x32, 0x03, 0xad, 0xff, 0x38, 0x01, 0xcc, 0xff,
    0x0b, 0x00, 0x26, 0xff, 0x07, 0x01, 0x1a, 0xff, 0x35, 0x00, 0xc1, 0xff,
    0x35, 0x00, 0xbb, 0xff, 0x07, 0x00, 0x1b, 0xff, 0x84, 0x8a, 0x97, 0xff,
    0xa5, 0x95, 0xed, 0xff, 0x59, 0x1c, 0xf8, 0xff, 0x31, 0x00, 0xb6, 0xff,
    0x31, 0x00, 0xb0, 0xff, 0x45, 0x00, 0xf5, 0xff, 0x42, 0x00, 0xf3, 0xff,
    0x2e, 0x00, 0xa7, 0xff, 0xc4, 0xcb, 0xde, 0xff, 0x9c, 0x81, 0xff, 0xff,
    0x53, 0x11, 0xfc, 0xff, 0x44, 0x00, 0xff, 0xff, 0x46, 0x00, 0xff, 0xff,
    0x45, 0x00, 0xfd, 0xff, 0x44, 0x01, 0xfe, 0xff, 0x43, 0x00, 0xef, 0xff,
    0xa4, 0x90, 0xdb, 0xff, 0x60, 0x27, 0xfe, 0xff, 0x43, 0x00, 0xfd, 0xff,
    0x46, 0x00, 0xfe, 0xff, 0x47, 0x01, 0xff, 0xff, 0x46, 0x00, 0xfe, 0xff,
    0x46, 0x00, 0xfe, 0xff, 0x43, 0x00, 0xef, 0xff, 0x58, 0x3a, 0x93, 0xff,
    0x64, 0x2b, 0xec, 0xff, 0x4b, 0x07, 0xfe, 0xff, 0x48, 0x00, 0xfe, 0xff,
    0x46, 0x00, 0xfe, 0xff, 0x46, 0x00, 0xfe, 0xff, 0x43, 0x00, 0xf0, 0xff,
    0x2d, 0x01, 0xa2, 0xff, 0x09, 0x07, 0x0d, 0xff, 0x58, 0x39, 0x94, 0xff,
    0x50, 0x0f, 0xf4, 0xff, 0x44, 0x01, 0xfe, 0xff, 0x46, 0x00, 0xfe, 0xff,
    0x44, 0x00, 0xf7, 0xff, 0x2d, 0x00, 0xa3, 0xff, 0x05, 0x00, 0x15, 0xff,
    0x00, 0x00, 0x00, 0xff, 0x08, 0x02, 0x13, 0xff, 0x30, 0x00, 0xac, 0xff,
    0x44, 0x00, 0xf7, 0xff, 0x45, 0x01, 0xf8, 0xff, 0x32, 0x00, 0xb8, 0xff,
    0x08, 0x00, 0x1e, 0xff, 0x00, 0x00, 0x00, 0xff, 0x01, 0x01, 0x01, 0xff,
    0x00, 0x00, 0x00, 0xff, 0x0a, 0x00, 0x25, 0xff, 0x3a, 0x00, 0xd4, 0xff,
    0x3e, 0x00, 0xe0, 0xff, 0x0e, 0x00, 0x30, 0xff, 0x00, 0x00, 0x00, 0xff,
    0x00, 0x00, 0x00, 0xff,
};

const uint8_t one_pixel[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void fill_white_color(uint16_t brightness) {
  // display_pixel_t sample_pixel;

  // for (uint8_t channel = 0; channel < COLOR_CHANNELS_PER_PIXEL; channel++)
  //   for (uint8_t offset = 0; offset < PIXEL_COLOR_DEPTH_BITS; offset++)
  //     // Send MSB first
  //     sample_pixel.data[channel * PIXEL_COLOR_DEPTH_BITS +
  //                       (PIXEL_COLOR_DEPTH_BITS - 1 - offset)] =
  //         (brightness >> offset) & BIT_DATA;

  for (uint8_t matrix_frame_parallel_row = 0;
       matrix_frame_parallel_row < ROWS_PER_FRAME;
       matrix_frame_parallel_row++) {

    display_row_t *fb_row_malloc_ptr = (display_row_t *)
        matrix_row_framebuffer_malloc[matrix_frame_parallel_row];

    // uint8_t address = (matrix_frame_parallel_row & 0x01) << 6 |
    // (matrix_frame_parallel_row & 0x02) << 4 | (matrix_frame_parallel_row &
    // 0x04) << 2;
    uint8_t address = matrix_frame_parallel_row << 4;
    memset(fb_row_malloc_ptr, address, sizeof(display_row_t));

    for (uint8_t channel = 0; channel < COLOR_CHANNELS_PER_PIXEL; channel++) {
      for (uint8_t pixel = 0; pixel < PIXELS_PER_ROW; pixel++) {
        uint16_t pixel_address = matrix_frame_parallel_row * 8 + pixel;

        display_pixel_t sample_pixel;

        for (uint8_t offset = 0; offset < PIXEL_COLOR_DEPTH_BITS; offset++)
          // Send MSB first
          sample_pixel.bits[PIXEL_COLOR_DEPTH_BITS - 1 - offset] =
              (((uint16_t)one_pixel[pixel_address]) >> offset) & BIT_DATA;

        display_pixel_t *bits_malloc_ptr =
            (display_pixel_t *)&fb_row_malloc_ptr->channels[channel].pixels[pixel];
        for (uint8_t offset = 0; offset < PIXEL_COLOR_DEPTH_BITS; offset++) {
          bits_malloc_ptr->bits[offset * 2] |= sample_pixel.bits[offset];
          bits_malloc_ptr->bits[offset * 2 + 1] |= sample_pixel.bits[offset] | BIT_SCLK;
        }
      }
    }

    fb_row_malloc_ptr->xlat_bits[0] |= BIT_XLAT;
    fb_row_malloc_ptr->xlat_bits[1] |= BIT_XLAT;
    fb_row_malloc_ptr->xlat_bits[2] |= BIT_XLAT;
    fb_row_malloc_ptr->xlat_bits[3] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[4] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[5] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[6] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[7] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[8] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[9] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[10] |= 0x00;
    fb_row_malloc_ptr->xlat_bits[11] |= 0x00;
  }
}

bool initialize_display(void) {
  allocate_dma_memory();
  configure_dma();

  clear_buffer();
  set_address_and_latch();

  fill_white_color(4095);

  return true;
}
