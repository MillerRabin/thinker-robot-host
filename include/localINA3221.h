#pragma once

#include <Wire.h>
#include <INA3221.h>
#include "callback.h"
#include "config.h"
#include "power.h"
#include "i2cScan.h"

class LocalINAData {
  public : 
    Power powerCh1;
    Power powerCh2;
    Power powerCh3;
    Power getPowerCh1() { return powerCh1; };
    Power getPowerCh2() { return powerCh2; };
    Power getPowerCh3() { return powerCh3; };
    uint64_t serializeCPULine();
    uint64_t serializeEnginesLine();
};

class LocalINA3221 {
private:
  static INA3221 ina;
  static void loop(void* instance);
  static DetectorsCallback callback;
  static SemaphoreHandle_t loopMutex;
  static LocalINAData data;
public:  
  LocalINAData getLocalData();
  void readAll();
  void begin(TwoWire &wire, DetectorsCallback callback);
};