#pragma once

#include <Wire.h>
#include <SPI.h>

#include "armPlatform.h"
#include "armShoulder.h"
#include "armElbow.h"
#include "armWrist.h"
#include "armClaw.h"
#include "twai.h"
#include "powerManagement.h"
#include "AsyncJson.h"
#include "responseStatus.h"

class ArmStatus {
  public: 
    bool canSendOK;
    bool shoulderQuaternionOK;
    bool elbowQuaternionOK;
    bool wristQuaternionOK;
    bool clawQuaternionOK;
    bool clawRangeOK;
    uint64_t shoulderStatuses;
};

class Arm {
  private:
    static TWAI twai;        
    static void twaiCallback(CanFrame frame);
    static void twaiErrorCallback(CanFrame frame, int code);
    static void detectorsCallback(uint32_t id, uint64_t data);
    static void loop(void* parameters);
    static bool getFloat(JsonObject jsonObj, const char* key, float& result);
    static bool getBool(JsonObject jsonObj, const char* key, bool& result);
  public:
    static ArmStatus status;
    static ArmPlatform platform;
    static ArmShoulder shoulder;
    static ArmElbow elbow;
    static ArmWrist wrist;
    static ArmClaw claw;
    static PowerManagement powerManagement;
    static void begin(TwoWire& wire, SPIClass& spi);
    static void set(JsonObject data);
    static void setRotate(JsonObject data);
    static StatusResponse upgrade(JsonObject data);
};