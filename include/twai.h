#pragma once

#include <Arduino.h>
#include "config.h"
#include <ESP32-TWAI-CAN.hpp>
#include <map>

typedef void (* TWAICallback)( CanFrame frame );
typedef void (* TWAIErrorCallback)( CanFrame frame, int code );
typedef std::map<uint32_t, CanFrame> CanMap;

class TWAI {
  private:
    const static uint rxPin;
    const static uint txPin;
    static CanFrame rxFrame;
    static TWAICallback callback;
    static TWAIErrorCallback errorCallback;
    static void loop(void* parameters);
    static bool read();    
    static CanMap canSendMap;    
    static void loopTask(void* parameters);
    static void sendTask(void* parameters);
    static void receiveTask(void* parameters);
    static QueueHandle_t receiveQueue;
    static volatile uint32_t droppedFramesCount;
  public:
    static SemaphoreHandle_t sendSemaphore;
    static uint32_t getDroppedFrames();
    static void begin(TWAICallback callback, TWAIErrorCallback errorCallback);    
    static bool sendData(uint32_t id, uint8_t* data);
};