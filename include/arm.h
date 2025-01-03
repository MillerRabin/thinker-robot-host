#pragma once

#include <Wire.h>
#include <SPI.h>

#include "armPlatform.h"
#include "armShoulder.h"
#include "armElbow.h"
#include "armWrist.h"
#include "twai.h"
#include "powerManagement.h"
#include "AsyncJson.h"

class Arm {
  private:
    static TWAI twai;    
    static PowerManagement powerManagement;    
    static void twaiCallback(CanFrame frame);
    static void detectorsCallback(uint32_t id, uint64_t data);
    static void loop(void* parameters);
    static bool getFloat(JsonObject& jsonObj, const char* key, float& result);
  public:    
    static ArmPlatform platform;
    static ArmShoulder shoulder;
    static ArmElbow elbow;
    static ArmWrist wrist;
    static void begin(TwoWire& wire, SPIClass& spi);
    static void set(JsonObject& data);
    static void setRotate(JsonObject& data);
};