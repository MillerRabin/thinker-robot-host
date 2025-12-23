#pragma once

#include <SparkFun_BNO080_Arduino_Library.h>
#include <Arduino.h>
#include <Wire.h>
#include "callback.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "accuracy.h"
#include "quaternion.h"
#include "config.h"

class LocalBNOData {
  public:
    Quaternion rawQuaternion;
    Accelerometer rawAccelerometer;
    Gyroscope rawGyroscope;
    Accuracy rawAccuracy;
    Quaternion getQuaternion() { return rawQuaternion; };
    Accelerometer getAccelerometer() { return rawAccelerometer; };
    Gyroscope getGyroscope() { return rawGyroscope; };
    Accuracy getAccuracy() { return rawAccuracy; };
};

class LocalBNO {
  private:
    static BNO080 bno;
    static unsigned int imuIntPin;
    static unsigned int imuRstPin;
    static void IRAM_ATTR interruptHandler();
    static TaskHandle_t taskHandle;
    static const uint32_t notificationIndex;    
    static void loop(void* parameters);
    static DetectorsCallback callback;
    static SemaphoreHandle_t loopMutex;
    static bool initBNO(); 
    static SPIClass* spi;
    static void hardReset();
    static LocalBNOData data;
  public:    
    static void begin(SPIClass& spi, DetectorsCallback callback);
    static Quaternion rotateQuatenion;
    static IMUQuaternion quaternion;
    LocalBNOData getLocalData();
    static Accelerometer accelerometer;
    static Gyroscope gyroscope;
    static Accuracy accuracy;
};