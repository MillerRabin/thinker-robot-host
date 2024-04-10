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
    static void twaiCallback(CanFrame frame);
    static void loop(void* parameters);
    
  public:      
    static void begin(TwoWire& wire);
};