#pragma once

#include "remoteBNO.h"

class ArmWrist {  
  public:
    static RemoteBNO imu;
    // The wrist's pitch and side bend as the module holds them (CAN_WRIST_ANGLES), deg; at = millis()
    // when heard, 0 never.
    static void setAngles(const uint8_t data[8]);
    static void readAngles(float &pitchDeg, float &bendDeg, uint32_t &at);
};
