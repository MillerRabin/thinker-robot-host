#include "twai.h"

CanFrame TWAI::rxFrame;
const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;

bool TWAI::sendData(uint32_t id, uint8_t* data) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = id;
  obdFrame.extd = 0;
  obdFrame.data_length_code = 8;
  memcpy(obdFrame.data, data, 8);
  return ESP32Can.writeFrame(obdFrame);
}

void TWAI::begin(TWAICallback callback) {
  TWAI::callback = callback;
  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), txPin, rxPin, 5, 5)) {
    Serial.println("CAN bus started!");
  }
  else {
    Serial.println("CAN bus failed!");
  }
  xTaskCreate(
    loop,
    "TWAI::loop",
    4096,
    NULL,
    5,
    NULL
  );
}

void TWAI::loop(void* parameters) {
  byte dataReady = false;
  while (true) {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
      callback(rxFrame);
    } else {
      vTaskDelay(pdMS_TO_TICKS(2000));
    }    
  }
}

