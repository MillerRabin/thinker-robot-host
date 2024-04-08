#pragma once

#include "armShoulder.h"
#include "bno080.h"

class Arm {
    static BNO085 imu;
    ArmShoulder shoulder;
}