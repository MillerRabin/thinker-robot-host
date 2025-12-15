#pragma once

#include <SparkFun_BNO080_Arduino_Library.h>
#include <Arduino.h>
#include <Wire.h>
#include "accelerometer.h"
#include "gyroscope.h"

#include "structures.h"
#include "quaternion.h"
#include "config.h"

class LocalBNO {
  private:
    static BNO080 bno;
    static uint imuIntPin;
    static void interruptHandler();    
    static TaskHandle_t taskHandle;
    static const uint32_t notificationIndex;
    static void printData();
    static void loop(void* parameters);
    static DetectorsCallback callback;
    static SemaphoreHandle_t loopMutex;
    static void initBNO();
    static Quaternion lastQuaternion;
  public:    
    static void begin(SPIClass& spi, DetectorsCallback callback);
    static Quaternion rotateQuatenion;
    static IMUQuaternion quaternion;
    static Quaternion getQuaternion();
    static Accelerometer accelerometer;
    static Gyroscope gyroscope;
    static Accuracy accuracy;
};