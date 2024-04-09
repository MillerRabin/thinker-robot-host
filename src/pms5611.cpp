
#include "pms5611.h"

MS5611 PMS5611::ms(0x77);

volatile float PMS5611::pressure;
volatile float PMS5611::temperature;

void PMS5611::begin() {
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
    int result = ms.read();
    if (result != MS5611_READ_OK) {
      Serial.print("Error in read: ");
      Serial.println(result);
    } else {      
      temperature = ms.getTemperature();      
      pressure = ms.getPressure();      
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

