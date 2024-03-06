#include "altitude.h"

const char* Altitude::TAG = "Altitude";

float convertToAltitude(int32_t pressure) {
    float A = (1 - pow(pressure/(double)1013.250, 0.190295)) * 44330.0;    
    return((float)A);
} 

void task(void *pvParameters)
{
  ms5611_t dev1 = {(i2c_port_t)0};
  ms5611_t dev2 = {(i2c_port_t)0};

  ESP_ERROR_CHECK(ms5611_init_desc(&dev1, MS5561_ADDR, (i2c_port_t)0, FIRST_LINE_SDA, FIRST_LINE_SCL));
  ESP_ERROR_CHECK(ms5611_init_desc(&dev2, MS5561_ADDR, (i2c_port_t)1, SECOND_LINE_SDA, SECOND_LINE_SCL));
  ESP_ERROR_CHECK(ms5611_init(&dev1, MS5611_OSR_4096));
  ESP_ERROR_CHECK(ms5611_init(&dev2, MS5611_OSR_4096));

  float temperature1;
  int32_t pressure1;
  float temperature2;
  int32_t pressure2;
  esp_err_t res1;
  esp_err_t res2;

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    res1 = ms5611_get_sensor_data(&dev1, &pressure1, &temperature1);    
    if (res1 != ESP_OK)
    {
      ESP_LOGE(Altitude::TAG, "Device 1 Temperature/pressure reading failed: %d (%s)", res1, esp_err_to_name(res1));
      continue;
    }

    res2 = ms5611_get_sensor_data(&dev2, &pressure2, &temperature2);
    if (res2 != ESP_OK)
    {
      ESP_LOGE(Altitude::TAG, "Device 2 Temperature/pressure reading failed: %d (%s)", res2, esp_err_to_name(res2));
      continue;
    }
    
    const int altitude1 = floor(convertToAltitude(pressure1));
    const int altitude2 = floor(convertToAltitude(pressure2));
    int32_t pd = pressure1 - pressure2 + 700;
    const int ad = altitude1 - altitude2;
    ESP_LOGI(Altitude::TAG, "Device 1 Pressure: %" PRIi32 " Pa, Temperature: %.2f C, Altitude: %d", pressure1, temperature1, altitude1);
    ESP_LOGI(Altitude::TAG, "Device 2 Pressure: %" PRIi32 " Pa, Temperature: %.2f C, Altitude: %d", pressure2, temperature2, altitude2);
    ESP_LOGI(Altitude::TAG, "Pressure delta: %" PRIi32 " Pa, Altitude delta: %d\n", pd, ad);
  }
}

Altitude::Altitude() {
  ESP_ERROR_CHECK(i2cdev_init());
  xTaskCreatePinnedToCore(task, "altitude", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);  
}
