#pragma once

#include "structures.h"

class RemoteBNO {
  private:
    int16_t rotationVector_Q1 = 14;
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
  public:
    static Quaternion quaternion;
};