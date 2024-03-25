#include "wifi/wifi.h"
#include "altitude/altitude.h"
#include "bus/bus.h"
#include "powerManagement/powerManagement.h"
#include <esp_log.h>

extern "C" void app_main(void)
{      
  //system_init();
  //esp_err_t start_rest_server();
  ESP_LOGI("Main", "Init Power");
  PowerManagement::init();
  //ESP_LOGI("Main", "Enable camera");
  //PowerManagement::enableCamera();
  ESP_LOGI("Main", "Enable engines");
  PowerManagement::enableEngines();
  ESP_LOGI("Main", "Finishing");
  //Altitude alt;
  // i2c_detect();
  /*ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(wifi_connect());
  ESP_ERROR_CHECK(start_rest_server());*/
  twai_initialize();
  //vTaskStartScheduler();
}
