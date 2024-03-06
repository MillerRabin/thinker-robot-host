#pragma once

#include <inttypes.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <ms5611.h>

#define FIRST_LINE_SDA (gpio_num_t)4
#define FIRST_LINE_SCL (gpio_num_t)5
#define SECOND_LINE_SDA (gpio_num_t)6
#define SECOND_LINE_SCL (gpio_num_t)7
#define MS5561_ADDR 0x77

class Altitude {    
  public:
    static const char *TAG;    
    Altitude();    
};
