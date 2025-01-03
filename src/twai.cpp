#include "twai.h"

const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;
SemaphoreHandle_t TWAI::receiveSemaphore;
SemaphoreHandle_t TWAI::sendSemaphore;
CanFrame TWAI::rxFrame = { 0 };

uint8_t notifyQueueStorage[1];


CanMap TWAI::canReceiveMap;
CanMap TWAI::canSendMap;

bool TWAI::sendData(uint32_t id, uint8_t* data) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = id;
  obdFrame.data_length_code = 8;
  obdFrame.extd = 0;
  memcpy(obdFrame.data, data, 8);
  if (xSemaphoreTake(TWAI::sendSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
    Serial.printf("Can't obtain receiveSemaphore in sendData\n"); 
    return false;
  };  
  canSendMap[id] = obdFrame;
  xSemaphoreGive(TWAI::sendSemaphore);
  return true;  
}

void TWAI::begin(TWAICallback callback) {
  TWAI::callback = callback;  
  TWAI::receiveSemaphore = xSemaphoreCreateMutex();
  TWAI::sendSemaphore = xSemaphoreCreateMutex();  
  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), txPin, rxPin, 5, 5)) {
    Serial.println("CAN bus started!");
  }
  else {
    Serial.println("CAN bus failed!");
  }
  xTaskCreate(loopTask, "TWAI::loop", 4096, NULL, 5, NULL);
  xTaskCreate(receiveTask, "TWAI::receive", 4096, NULL, 5, NULL);
  xTaskCreate(sendTask, "TWAI::send", 4096, NULL, 5, NULL);
}

bool TWAI::read() {  
  return ESP32Can.readFrame(rxFrame, 100);      
}

void TWAI::receiveTask(void* parameters) {    
  while (true) {    
    bool res = read();
    if (res) {
      if (xSemaphoreTake(TWAI::receiveSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.printf("Can't obtain receiveSemaphore in receiveTask\n"); 
        continue;
      };
      canReceiveMap[rxFrame.identifier] = rxFrame;
      xSemaphoreGive(TWAI::receiveSemaphore);
    }        
  }
}

void TWAI::sendTask(void* parameters) {  
  while (true) {
    if (xSemaphoreTake(TWAI::sendSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.printf("Can't obtain receiveSemaphore in sendTask\n"); 
        continue;
      };
    for(auto item: canSendMap) {
      CanFrame frame = item.second;      
      bool res = ESP32Can.writeFrame(frame);
    }
    //canSendMap.clear();
    xSemaphoreGive(TWAI::sendSemaphore);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TWAI::loopTask(void* parameters) {  
  Serial.printf("Loop task started\n");      
  while (true) {            
    if (xSemaphoreTake(TWAI::receiveSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.printf("Can't obtain receiveSemaphore in loopTask\n"); 
        continue;
      };
    for(auto item: canReceiveMap) {
      CanFrame frame = item.second;      
      callback(frame);
    }
    //canReceiveMap.clear();
    xSemaphoreGive(TWAI::receiveSemaphore);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
