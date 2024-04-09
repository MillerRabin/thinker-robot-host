#include "twai.h"

CanFrame TWAI::rxFrame;
const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;

/*void sendObdFrame(uint8_t obdId) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = 0x7DF; // Default OBD2 address;
  obdFrame.extd = 0;
  obdFrame.data_length_code = 8;
  obdFrame.data[0] = 2;
  obdFrame.data[1] = 1;
  obdFrame.data[2] = obdId;
  obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
  obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
  obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
  obdFrame.data[6] = 0xAA;
  obdFrame.data[7] = 0xAA;
    // Accepts both pointers and references
    ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}*/

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

