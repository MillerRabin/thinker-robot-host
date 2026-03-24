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

using Parser = int16_t (*)(float);

class ArmStatus {
  public: 
    bool canSendOK;
    bool shoulderQuaternionOK;
    bool elbowQuaternionOK;
    bool wristQuaternionOK;
    bool clawQuaternionOK;
    bool clawRangeOK;
    uint64_t shoulderStatuses;
    uint64_t elbowStatuses;
    uint64_t wristStatuses;
    uint64_t clawStatuses;
};

union ArmDataFrame
{
  struct {
    int16_t param1;
    int16_t param2;
    int16_t param3;
    int16_t param4;
  } values;
  uint8_t bytes[8];
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
    static void setPowerState(JsonObject data);   
    static bool sendArmData(JsonObject data, const char *key1,
                            int16_t (*parser1)(float), const char *key2,
                            int16_t (*parser2)(float), const char *key3,
                            int16_t (*parser3)(float), const char *key4,
                            int16_t (*parser4)(float),
                            const uint32_t canMessage);
    public:
      static ArmStatus status;
      static ArmPlatform platform;
      static ArmShoulder shoulder;
      static ArmElbow elbow;
      static ArmWrist wrist;
      static ArmClaw claw;
      static PowerManagement powerManagement;
      static void begin(TwoWire & wire);
      static void set(JsonObject data);
      static void setRotate(JsonObject data);
      static StatusResponse upgrade(JsonObject data);
      static void tare(JsonObject data);
    };