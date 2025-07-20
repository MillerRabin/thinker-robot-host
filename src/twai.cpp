#include "twai.h"

const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;
TWAIErrorCallback TWAI::errorCallback;
SemaphoreHandle_t TWAI::sendSemaphore;
QueueHandle_t TWAI::receiveQueue;
volatile uint32_t TWAI::droppedFramesCount = 0;

CanFrame TWAI::rxFrame = {0};

CanMap TWAI::canSendMap;

bool TWAI::sendData(uint32_t id, uint8_t* data) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = id;
  obdFrame.data_length_code = 8;
  obdFrame.extd = 0;
  memcpy(obdFrame.data, data, 8);
  if (xSemaphoreTake(TWAI::sendSemaphore, pdMS_TO_TICKS(CAN_SEND_WAIT_SEMAPHORE)) != pdTRUE) {
    Serial.println("Can't obtain sendSemaphore in sendTask");
    return false;
  };
  canSendMap[id] = obdFrame;
  xSemaphoreGive(TWAI::sendSemaphore);
  return true;  
}

void TWAI::begin(TWAICallback callback, TWAIErrorCallback errorCallback) {
  TWAI::callback = callback;
  TWAI::errorCallback = errorCallback;    
  TWAI::sendSemaphore = xSemaphoreCreateMutex();
  if (TWAI::sendSemaphore == NULL) {
    Serial.println("sendSemaphore allocation failed!");
    return;
  }

  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), txPin, rxPin, 10, 10)) {
    Serial.println("CAN bus started!");
  }
  else {
    Serial.println("CAN bus failed!");
    return;
  }

  TWAI::receiveQueue = xQueueCreate(CAN_RECEIVE_QUEUE_SIZE, sizeof(CanFrame));
  if (TWAI::receiveQueue == NULL)
  {
    Serial.println("Failed to create receiveQueue!");
    return;
  }

  xTaskCreate(loopTask, "TWAI::loop", 4096, NULL, 2, NULL);
  xTaskCreate(receiveTask, "TWAI::receive", 4096, NULL, 5, NULL);
  xTaskCreate(sendTask, "TWAI::send", 4096, NULL, 5, NULL);
}

bool TWAI::read() {  
  return ESP32Can.readFrame(rxFrame, 100);      
}

void TWAI::receiveTask(void *parameters) {
  while (true) {
    if (read())
    {      
      if (xQueueSend(receiveQueue, &rxFrame, 0) != pdTRUE) {
        TWAI::droppedFramesCount++;
        Serial.println("receiveQueue full, frame dropped");
      }
    }
  }
}

void TWAI::sendTask(void *parameters)
{
  printf("TWAI::Send task started\n");
  const int maxRetries = CAN_SEND_MAX_SEND_ATTEMPTS;

  while (true) {
    if (xSemaphoreTake(TWAI::sendSemaphore, pdMS_TO_TICKS(CAN_SEND_WAIT_SEMAPHORE)) != pdTRUE) {
      Serial.printf("Can't obtain sendSemaphore in sendTask\n");
      continue;
    }

    for (auto item : canSendMap) {
      CanFrame frame = item.second;
      bool sent = false;

      for (int attempt = 0; attempt < maxRetries; ++attempt) {
        sent = ESP32Can.writeFrame(frame);
        if (sent)
          break;
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (!sent) {
        errorCallback(frame, CAN_SEND_ERROR);
      } else {
        errorCallback(frame, CAN_SUCCESS);
      }
    }

    canSendMap.clear();
    xSemaphoreGive(TWAI::sendSemaphore);
    vTaskDelay(pdMS_TO_TICKS(CAN_SEND_LOOP_WAIT));
  }
}

void TWAI::loopTask(void *parameters) {
  Serial.printf("Loop task started\n");
  CanFrame frame;
  while (true) {
    if (xQueueReceive(receiveQueue, &frame, pdMS_TO_TICKS(CAN_RECEIVE_WAIT_MESSAGE)) == pdTRUE) {
      callback(frame);
    }
  }
}

uint32_t TWAI::getDroppedFrames() {
  return TWAI::droppedFramesCount;
}
