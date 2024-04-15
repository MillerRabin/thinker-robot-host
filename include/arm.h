#pragma once

#include <Wire.h>

#include "armPlatform.h"
#include "armShoulder.h"
#include "twai.h"
#include "powerManagement.h"
#include "AsyncJson.h"

class Arm {
  private:
    static TWAI twai;    
    static PowerManagement powerManagement;    
    static void twaiCallback(CanFrame frame);
    static void loop(void* parameters);
    static bool getFloat(JsonObject& jsonObj, const char* key, float& result);
  public:
    static ArmPlatform platform;
    static ArmShoulder shoulder;
    static void begin(TwoWire& wire);    
    static void set(JsonObject& data);
};