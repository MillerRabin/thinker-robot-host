#pragma once

#include "remoteWitmotion.h"
#include "measureRange.h"
#include "barometer.h"

class ArmClaw {  
  public:
    static RemoteWitmotion imu;
    static MeasureRange range;
    static Barometer barometer;
};