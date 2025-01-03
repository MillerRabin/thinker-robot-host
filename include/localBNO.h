#pragma once

#include <SparkFun_BNO080_Arduino_Library.h>
#include <Arduino.h>
#include <Wire.h>

#include "structures.h"
#include "quaternion.h"
#include "config.h"

class LocalBNO {
  private:
    static BNO080 bno;
    static uint imuIntPin;
    static void interruptHandler();
    void printAccuracyLevel(byte accuracyNumber);    
    static TaskHandle_t taskHandle;
    static const uint32_t notificationIndex;
    static void printData();
    static void loop(void* parameters);
    static DetectorsCallback callback;
  public:
    static void begin(SPIClass& spi, DetectorsCallback callback);
    static Quaternion quaternion;
    Quaternion getQuaternion();
    static Accelerometer accelerometer;
    static Gyroscope gyroscope;
    static Accuracy accuracy;
};