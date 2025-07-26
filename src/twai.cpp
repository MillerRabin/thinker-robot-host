#include "twai.h"

const uint TWAI::rxPin = TWAI_RX_GPIO;
const uint TWAI::txPin = TWAI_TX_GPIO;
TWAICallback TWAI::callback;
TWAIErrorCallback TWAI::errorCallback;
SemaphoreHandle_t TWAI::sendSemaphore;
SemaphoreHandle_t TWAI::receiveSemaphore;

CanFrame TWAI::rxFrame = {0};

CanMap TWAI::canSendMap;
CanMap TWAI::canReceiveMap;

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

  TWAI::receiveSemaphore = xSemaphoreCreateMutex();
  if (TWAI::receiveSemaphore == NULL) {
    Serial.println("receiveSemaphore allocation failed!");
    return;
  }

  if (ESP32Can.begin(ESP32Can.convertSpeed(1000), txPin, rxPin, 10, 10)) {
    Serial.println("CAN bus started!");
  }
  else {
    Serial.println("CAN bus failed!");
    return;
  }

  xTaskCreate(callbackTask, "TWAI::callbackTask", 4096, NULL, 2, NULL);
  xTaskCreate(receiveTask, "TWAI::receive", 4096, NULL, 5, NULL);
  xTaskCreate(sendTask, "TWAI::send", 4096, NULL, 5, NULL);
}

void TWAI::receiveTask(void *parameters) {
  while (true) {
    if (ESP32Can.readFrame(rxFrame, 100)) {
      if (xSemaphoreTake(TWAI::receiveSemaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
        TWAI::canReceiveMap[rxFrame.identifier] = rxFrame;
        xSemaphoreGive(TWAI::receiveSemaphore);
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

void TWAI::callbackTask(void *parameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    CanMap localCopy;
    if (xSemaphoreTake(TWAI::receiveSemaphore, pdMS_TO_TICKS(CAN_RECEIVE_WAIT_TIMEOUT)) == pdTRUE) {
      localCopy = std::move(TWAI::canReceiveMap);
      TWAI::canReceiveMap.clear();
      xSemaphoreGive(TWAI::receiveSemaphore);
    }

    for (auto &item : localCopy) {
      callback(item.second);
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }
}