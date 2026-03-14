#pragma once

#include <Arduino.h>
#include "config.h"
#include "driver/twai.h"
#include "canRingBuffer.h"

typedef void (* TWAICallback)( CanFrame frame );
typedef void (* TWAIErrorCallback)( CanFrame frame, int code );

typedef enum : int32_t {
  TWAI_SUCCESS = 0,
  TWAI_STOP_FAILED = -1,
  TWAI_UNISTALL_FAILED = -2,
  TWAI_SEND_SEMAPHORE_ALLOCATION_FAILED = -3,
  TWAI_RECEIVE_SEMAPHORE_ALLOCATION_FAILED = -4,
  TWAI_INSTALL_FAILED = -5,
  TWAI_START_FAILED = -6,
  TWAI_INIT_TASK_CREATION_FAILED = -7,
  TWAI_CALLBACK_TASK_CREATION_FAILED = -8,
  TWAI_RECEIVE_TASK_CREATION_FAILED = -9,
  TWAI_SEND_TASK_CREATION_FAILED = -10,
  TWAI_WATCHDOG_TASK_CREATION_FAILED = -11,
  TWAI_EVENT_GROUP_CREATION_FAILED = -12,
  TWAI_WRITE_MUTEX_CREATION_FAILED = -13
} twai_status_t;

enum TwaiSpeed : uint8_t {
#if (SOC_TWAI_BRP_MAX > 256)
  TWAI_SPEED_1KBPS,
  TWAI_SPEED_5KBPS,
  TWAI_SPEED_10KBPS,
#endif
#if (SOC_TWAI_BRP_MAX > 128) || (CONFIG_ESP32_REV_MIN_FULL >= 200)
  TWAI_SPEED_12_5KBPS,
  TWAI_SPEED_16KBPS,
  TWAI_SPEED_20KBPS,
#endif
  TWAI_SPEED_100KBPS,
  TWAI_SPEED_125KBPS,
  TWAI_SPEED_250KBPS,
  TWAI_SPEED_500KBPS,
  TWAI_SPEED_800KBPS,
  TWAI_SPEED_1000KBPS,
  TWAI_SPEED_SIZE
};

class TWAI {
  private:
    const gpio_num_t rxPin;
    const gpio_num_t txPin;
    TwaiSpeed speed = TWAI_SPEED_1000KBPS;
    const uint txQueueSize = 10;
    const uint rxQueueSize = 10;
    TaskHandle_t initTaskHandle = NULL;
    TaskHandle_t callbackTaskHandle = NULL;
    TaskHandle_t receiveTaskHandle = NULL;
    TaskHandle_t sendTaskHandle = NULL;
    TaskHandle_t watchdogTaskHandle = NULL;
    const TWAICallback callback;
    const TWAIErrorCallback errorCallback;           
    static void callbackTask(void* instance);
    static void sendTask(void *instance);
    static void receiveTask(void* instance);
    static void initTask(void *instance);
    static void watchdogTask(void *instance);
    twai_status_t end();
    volatile bool reinitInProgress = false;
    SemaphoreHandle_t writeMutex = NULL;
    public:
      void requestReinit();
      twai_status_t init();
      CanRingBuffer receiveBuffer;
      CanFrame sendBuffer[CAN_TX_BUFFER_SIZE];
      EventGroupHandle_t events = NULL;
      bool isBusAlive();
      inline bool IRAM_ATTR readFrame(CanFrame *frame, TickType_t timeout = portMAX_DELAY) {
        if (!frame)
          return false;

        auto status = twai_receive(frame, timeout);
        if (status == ESP_OK)
          return true;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        return false;
      }

      inline bool IRAM_ATTR writeFrame(CanFrame *frame, TickType_t timeout = portMAX_DELAY) {
        if (reinitInProgress) {
          return false;
        }
          
        if ((frame) && twai_transmit(frame, timeout) == ESP_OK) {
          return true;
        }
        return false;                  
      }

      TWAI(const gpio_num_t rxPin, const gpio_num_t txPin,
           TWAICallback callback, TWAIErrorCallback errorCallback)
          : rxPin(rxPin), txPin(txPin), callback(callback),
            errorCallback(errorCallback) {}
      twai_status_t begin(TwaiSpeed twaiSpeed = TWAI_SPEED_1000KBPS);
      bool sendData(uint32_t id, uint8_t *data);
      ~TWAI();
    };