#pragma once

#include <REG.h>
#include <wit_c_sdk.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "callback.h"
#include "accelerometer.h"
#include "gyroscope.h"
#include "accuracy.h"
#include "quaternion.h"
#include "barometer.h"
#include "config.h"

class LocalWitmotionData {
public:
  Quaternion rawQuaternion;
  Accelerometer rawAccelerometer;
  Gyroscope rawGyroscope;  
  Barometer rawBarometer;
  Quaternion getQuaternion() { return rawQuaternion; };
  Accelerometer getAccelerometer() { return rawAccelerometer; };
  Gyroscope getGyroscope() { return rawGyroscope; };  
  Barometer getBarometer() { return rawBarometer; };
};

class LocalWitmotion {
private:  
  const uint memsRxPin;
  const uint memsTxPin;
  uint32_t lastReceiveTime = 0;
  uint32_t receiveTimeout = 1000;
  static LocalWitmotion* instance;
  static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
  static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
  static void Delayms(uint16_t usMs);  
  static void init(void *instance);
  static void readDetectorTask(void *instance);
  void readData();
  uint32_t autoScanSensor(void);
  static volatile bool dataAvailable;
  static uint32_t c_uiBaud[8];  
  void setBaudRate();    
  static SemaphoreHandle_t loopMutex;
  LocalWitmotionData data;
  LocalWitmotionData prevData;
public:
  HardwareSerial &port;  
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  Quaternion quaternion;
  Barometer barometer;
  TaskHandle_t initTaskHandle;
  TaskHandle_t readDetectorTaskHandle;
  const DetectorsCallback callback;
  LocalWitmotionData getLocalData();
  bool begin();
  void set6AxisMode();
  void set9AxisMode();  
  void setRefreshRate();
  void setBandwidth();
  void setContent();
  bool isPresent() { return (millis() - lastReceiveTime) < receiveTimeout; }
  LocalWitmotion(HardwareSerial& port, const uint memsRXPin, const uint memsTxPin, DetectorsCallback callback);    
};