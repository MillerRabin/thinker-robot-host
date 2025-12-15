#pragma once

#include <Arduino.h>
#include "qBase.h"

class Accelerometer : public QBase {
private:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  int16_t Q1 = 8;
public:
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  void set(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);  
  float witMotionX() { return x / 32768.0f * 16.0f * 9.80665f; };
  float witMotionY() { return y / 32768.0f * 16.0f * 9.80665f; };
  float witMotionZ() { return z / 32768.0f * 16.0f * 9.80665f; };
  float bnoX() { return qToFloat(x, Q1); };
  float bnoY() { return qToFloat(y, Q1); };
  float bnoZ() { return qToFloat(z, Q1); };
};
