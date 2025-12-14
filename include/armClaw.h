#pragma once

#include "remoteWitmotion.h"
#include "range.h"
#include "barometer.h"

class ArmClaw {  
  public:
    static RemoteWitmotion imu;
    static Range range;
    static Barometer barometer;
};