
#include "pms5611.h"

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
      if (barometer.set(ms.getPressure(), ms.getTemperature())) {        
        callback(CAN_PLATFORM_BAROMETER, barometer.serialize());
      }
        
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

