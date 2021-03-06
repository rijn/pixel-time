#include "errno.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

#include "display.h"
#include "display2812.h"
#include "smartconfig.h"

#include "adxl345.h"
#include "i2c.h"
#include "veml6030.h"

static const char *TAG = "main";

void app_main(void) {
  // TODO: Check OTA partition

  // Initialize.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // OTA app partition table has a smaller NVS partition size than the non-OTA
    // partition table. This size mismatch may cause NVS initialization to fail.
    // If this happens, we erase NVS partition and initialize NVS again.
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  initialize_display();
  // initialize_display_2812();

  // initialize_wifi();

  // Initialize sensors
  // initialize_adxl345();

  i2c_master_init();
  i2c_detect();

  initialize_adxl345();
  initialize_veml6030();

  // TODO: Submit OTA task
}
