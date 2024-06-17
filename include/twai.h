#pragma once

#include <Arduino.h>
#include "config.h"
#include <ESP32-TWAI-CAN.hpp>
#include <map>

typedef void (* TWAICallback)( CanFrame frame );
typedef std::map<uint32_t, CanFrame> CanMap;

class TWAI {
  private:
    const static uint rxPin;
    const static uint txPin;
    static CanFrame rxFrame;
    static TWAICallback callback;
    static void loop(void* parameters);
    static bool read();    
    static StaticQueue_t notifyQueue;
    static QueueHandle_t notifyQueueHandle;
    static CanMap canSendMap;
    static CanMap canReceiveMap;
    static void loopTask(void* parameters);
    static void sendTask(void* parameters);
    static void receiveTask(void* parameters);
  public:
    static SemaphoreHandle_t semaphore;
    static void begin(TWAICallback callback);    
    static bool sendData(uint32_t id, uint8_t* data);
};