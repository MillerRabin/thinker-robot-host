#pragma once

#include "structures.h"
#include "quaternion.h"
#include "accelerometer.h"
#include "gyroscope.h"

class RemoteWitmotion {
  private:
    int16_t rotationVector_Q1 = 14;  
  public:
    Quaternion getQuaternion();
    IMUQuaternion quaternion;
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Accuracy accuracy;
};