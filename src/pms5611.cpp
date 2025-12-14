
#include "pms5611.h"
#include "barometer.h"

MS5611 PMS5611::ms(0x77);
Barometer PMS5611::barometer;
DetectorsCallback PMS5611::callback;

void PMS5611::begin(TwoWire& wire, DetectorsCallback callback) {
  PMS5611::callback = callback;
  if (!ms.begin()) {
    Serial.println("MS5611 is not found.");
  }
  ms.reset(1);

  xTaskCreate(
    loop,
    "PMS5611::loop",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    NULL
  );
}

void PMS5611::loop(void* parameters) {  
  while (true) {
    ms.setOversampling(OSR_ULTRA_HIGH);
    int result = ms.read();
    if (result != MS5611_READ_OK) {
      Serial.print("Error in read: ");
      Serial.println(result);
    } else {
      uint16_t pressure = (uint16_t)(ms.getPressure() * 100);
      uint8_t temperature = (uint8_t)(ms.getTemperature());
      uint16_t height = (uint16_t)(44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903)));
      barometer.set(height, pressure, temperature);
      callback(CAN_PLATFORM_BAROMETER, barometer.serialize());
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

