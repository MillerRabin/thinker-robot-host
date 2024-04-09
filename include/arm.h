#pragma once

#include <Wire.h>

#include "armPlatform.h"
#include "armShoulder.h"
#include "twai.h"
#include "powerManagement.h"

class Arm {
  private:
    static TWAI twai;
    static ArmPlatform platform;
    static ArmShoulder shoulder;
    static PowerManagement powerManagement;
    static void getQuaternion(uint8_t data[8]);
    static void twaiCallback(CanFrame frame);
    
  public:      
    static void begin(TwoWire& wire);
};