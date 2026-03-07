#pragma once

#include "accuracy.h"
#include "quaternion.h"
#include "accelerometer.h"
#include "gyroscope.h"

class RemoteWitmotion {
  private:
    int16_t rotationVector_Q1 = 14;  
  public:
    Quaternion getQuaternion();
    Quaternion quaternion;
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Accuracy accuracy;
};