#include "sdkconfig.h"
#include "driver/gpio.h"
#include "wifi/wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "i2c/i2c.h"
#include "ms5611.h"
#include <math.h>
#include "altitude/altitude.h"
#include "twai/twai.h"

esp_err_t start_rest_server();

#define FIRST_LINE_SDA 4
#define FIRST_LINE_SCL 5
#define SECOND_LINE_SDA 6
#define SECOND_LINE_SCL 7
#define MS5561_ADDR 0x77

extern "C" void app_main(void)
{    
  twai_initialize();
  //Altitude alt;
  // i2c_detect();
  /*ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(wifi_connect());
  ESP_ERROR_CHECK(start_rest_server());*/
}
