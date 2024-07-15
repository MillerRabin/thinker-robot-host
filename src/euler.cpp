#include "euler.h"

Euler::Euler(const float roll, const float pitch, const float yaw) : 
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

const float Euler::getRollAngle() { return roll * 180.0 / PI; }
const float Euler::getPitchAngle() { return pitch * 180.0 / PI; }
const float Euler::getYawAngle() { return yaw * 180.0 / PI; }

const float Euler::getYawDegree() { 
  const float angle = getYawAngle();
  const float roll = getRollAngle();
  const float k = (roll > 0) ? 0 : 180;
  return k - angle;  
}
