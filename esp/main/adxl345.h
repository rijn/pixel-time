#ifndef __ADXL345_H__
#define __ADXL345_H__

#include "esp_event_base.h"

typedef enum {
  SENSOR_EVENT_ACCELERATION_UPDATED,
} sensor_event_t;

ESP_EVENT_DECLARE_BASE(SENSOR_EVENT);

void initialize_adxl345(void);

#endif
