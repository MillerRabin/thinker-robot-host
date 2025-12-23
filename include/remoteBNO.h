#pragma once

#include "accuracy.h"
#include "quaternion.h"
#include "accelerometer.h"
#include "gyroscope.h"

class RemoteBNO {
  private:
    int16_t rotationVector_Q1 = 14;  
  public:
    Quaternion getQuaternion();
    IMUQuaternion quaternion;
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Accuracy accuracy;
};