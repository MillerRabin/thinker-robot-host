#include "twai.h"

const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;
SemaphoreHandle_t TWAI::semaphore;
StaticQueue_t TWAI::notifyQueue;
QueueHandle_t TWAI::notifyQueueHandle;
CanFrame TWAI::rxFrame = { 0 };

#define CAN_PLATFORM_QUATERNION 0x10
#define CAN_PLATFORM_ACCELEROMETER 0x11
#define CAN_PLATFORM_GYROSCOPE 0x12
#define CAN_PLATFORM_ACCURACY 0x13
#define CAN_PLATFORM_BAROMETER 0x14

#define CAN_SHOULDER_QUATERNION 0x20
#define CAN_SHOULDER_ACCELEROMETER 0x21
#define CAN_SHOULDER_GYROSCOPE 0x22
#define CAN_SHOULDER_ACCURACY 0x23
#define CAN_SHOULDER_SET_YZ_DEGREE 0x24
#define CAN_SHOULDER_SET_ROTATE_RADIAN 0x25
uint8_t notifyQueueStorage[1];


CanMap TWAI::canReceiveMap;
CanMap TWAI::canSendMap;

bool TWAI::sendData(uint32_t id, uint8_t* data) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = id;
  obdFrame.data_length_code = 8;
  obdFrame.extd = 0;
  memcpy(obdFrame.data, data, 8);
  canSendMap[id] = obdFrame;
  return true;  
}

void TWAI::begin(TWAICallback callback) {
  TWAI::callback = callback;
  notifyQueueHandle = xQueueCreateStatic(1, 1, notifyQueueStorage, &notifyQueue);  
  TWAI::semaphore = xSemaphoreCreateMutex();
  if(TWAI::semaphore == NULL)
    Serial.printf("TWAI SendTesk Semaphore not created\n");  
  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), txPin, rxPin, 5, 5)) {
    Serial.println("CAN bus started!");
  }
  else {
    Serial.println("CAN bus failed!");
  }
  xTaskCreate(loopTask, "TWAI::loop", 4096, NULL, 5, NULL);
  xTaskCreate(receiveTask, "TWAI::receive", 1024, NULL, 5, NULL);
  xTaskCreate(sendTask, "TWAI::send", 1024, NULL, 5, NULL);
}

bool TWAI::read() {
  xSemaphoreTake(TWAI::semaphore, portMAX_DELAY);
  bool res = ESP32Can.readFrame(rxFrame, 100);  
  xSemaphoreGive(TWAI::semaphore);
  return res;
}

void TWAI::receiveTask(void* parameters) {  
  uint8_t dataReady = 1;
  while (true) {    
    bool res = read();
    if (res)
      canReceiveMap[rxFrame.identifier] = rxFrame;
    xQueueSend(notifyQueueHandle, &dataReady, 0);   
  }
}

void TWAI::sendTask(void* parameters) {  
  while (true) {
    for(auto item: canSendMap) {
      CanFrame frame = item.second;
      bool res = ESP32Can.writeFrame(frame);
    }
    canSendMap.clear();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TWAI::loopTask(void* parameters) {  
  Serial.printf("Loop task started\n");    
  uint8_t dataReady = 0;
  while (true) {    
    if (xQueueReceive(notifyQueueHandle, &dataReady, pdMS_TO_TICKS(2000)) != pdTRUE) {
      Serial.printf("No data from From Can Bus\n");    
      continue;
    }

    for(auto item: canReceiveMap) {
      CanFrame frame = item.second;      
      callback(frame);            
    }
    canReceiveMap.clear();
  }
}
