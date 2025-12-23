
#include "pms5611.h"
#include "barometer.h"

MS5611 PMS5611::ms(0x77);
Barometer PMS5611::barometer;
DetectorsCallback PMS5611::callback;
SemaphoreHandle_t PMS5611::loopMutex;
LocalMSData PMS5611::data;

void PMS5611::begin(TwoWire& wire, DetectorsCallback callback) {
  PMS5611::callback = callback;
  PMS5611::loopMutex = xSemaphoreCreateMutex();
  if (!ms.begin()) {
    Serial.println("MS5611 is not found.");
  }
  ms.reset(1);

  xTaskCreate(loop, "PMS5611::loop", 4096, NULL, tskIDLE_PRIORITY, NULL);
}

void PMS5611::loop(void* parameters) {  
  while (true) {
    ms.setOversampling(OSR_ULTRA_HIGH);
    int result = ms.read();
    if (result != MS5611_READ_OK) {
      Serial.print("Error in read: ");
      Serial.println(result);
    } else {
      float pressure = ms.getPressure();      
      float pressure_hPa = pressure / 100.0;
      float seaLevelPressure = 1013.25;
      uint16_t temperature = ms.getTemperature();
      uint32_t height = 44330.0 * (1.0 - pow(pressure_hPa / seaLevelPressure, 0.1903));      
      barometer.set(height, temperature);
      callback(CAN_PLATFORM_BAROMETER, barometer.serialize());
      if (xSemaphoreTake(loopMutex, pdMS_TO_TICKS(10))) {
        data.rawBarometer = Barometer(barometer);
        xSemaphoreGive(loopMutex);
      } else {
        Serial.println("Can't obtain loopMutex semaphore lock");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

LocalMSData PMS5611::getLocalData() {
  if (xSemaphoreTake(loopMutex, pdMS_TO_TICKS(IMU_WAIT_MUTEX))) {
    xSemaphoreGive(loopMutex);
  } else {
    Serial.println("Can't obtain loop semaphore lock");
  }
  return data;
}
