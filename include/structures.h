
#pragma once

#include <Arduino.h>

typedef void (* DetectorsCallback)(uint32_t id, uint64_t data);

class Accuracy {
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;  
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy);
};

class Accelerometer {
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);
};

class Gyroscope {
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ);
};