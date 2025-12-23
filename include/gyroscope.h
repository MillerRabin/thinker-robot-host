#pragma once

#include <Arduino.h>
#include "qBase.h"

class Gyroscope : public QBase {
  public:
    uint16_t x;
    uint16_t y;
    uint16_t z;
    int16_t Q1 = 9;
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    void set(uint16_t rawX, uint16_t rawY, uint16_t rawZ);
    float witMotionX() { return x / 32768.0f * 2000.0f; };
    float witMotionY() { return y / 32768.0f * 2000.0f; };
    float witMotionZ() { return z / 32768.0f * 2000.0f; };
    float bnoX() { return qToFloat(x, Q1); };
    float bnoY() { return qToFloat(y, Q1); };
    float bnoZ() { return qToFloat(z, Q1); };
};
