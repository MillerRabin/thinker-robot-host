#pragma once

#include <SparkFun_BNO080_Arduino_Library.h>
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

class LocalBNO {
  private:
    static BNO080 bno;
    static uint imuIntPin;
    static void interruptHandler();
    void printAccuracyLevel(byte accuracyNumber);    
    static uint8_t queueStorage[1];
    static StaticQueue_t xStaticQueue;
    static QueueHandle_t queueHandle;
    static void printData();
    static void loop(void* parameters);
  public:
    static void begin(TwoWire& wire);
};