#pragma once

#include "structures.h"
#include "quaternion.h"

class RemoteBNO {
  private:
    int16_t rotationVector_Q1 = 14;    
  public:
    Quaternion quaternion;
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Accuracy accuracy;
};