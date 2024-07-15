#pragma once

#include <Arduino.h>

class StructureBase {
  public:
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
    uint16_t floatToQ(float q, uint8_t qPoint);    
};
