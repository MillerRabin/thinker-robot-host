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
    BNO080 bno;
    const unsigned int imuIntPin;
    const unsigned int imuRstPin;
    static void IRAM_ATTR interruptHandler();
    static TaskHandle_t taskHandle;
    static const uint32_t notificationIndex;    
    static void loop(void* instance);
    DetectorsCallback callback;
    static SemaphoreHandle_t loopMutex;
    bool initBNO(); 
    SPIClass* spi;
    void hardReset();
    LocalBNOData data;
  public:    
    void begin(SPIClass& spi, DetectorsCallback callback);
    Quaternion rotateQuatenion = Quaternion(0.0f, 0.0f, 0.0f, 1.0f); // East North Up;
    Quaternion quaternion;
    LocalBNOData getLocalData();
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Accuracy accuracy;
    void tare(uint8_t axisMask);
    void saveTare() {
      bno.saveTare();
    }
    void clearTare() {
      bno.clearTare();
    }
    LocalBNO(const unsigned int imuIntPin, const unsigned int imuRstPin) : imuIntPin(imuIntPin), imuRstPin(imuRstPin) {};
};