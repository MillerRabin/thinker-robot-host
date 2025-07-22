#pragma once

#include <Arduino.h>

class StructureBase {
  public:
    static float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
    static uint16_t floatToQ(float q, uint8_t qPoint);
    static uint32_t floatToQ32(float q, uint8_t qPoint);
  };
