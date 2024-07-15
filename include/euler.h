#pragma once

#include <Arduino.h>

class Euler {
  public:
    Euler(const float roll, const float pitch, const float yaw);
    const float yaw;
    const float pitch;
    const float roll;
    const float getRollAngle();
    const float getPitchAngle();
    const float getYawAngle();
    const float getYawDegree();
};
