#pragma once

#include <Wire.h>
#include <MS5611.h>
#include "barometer.h"
#include "callback.h"
#include "config.h"

class LocalMSData {
public:
  Barometer rawBarometer;
  Barometer getBarometer() { return rawBarometer; };  
};

class PMS5611 {
    private:
      static MS5611 ms;
      static void loop(void* parameters);      
      static DetectorsCallback callback;
      static SemaphoreHandle_t loopMutex;
      static LocalMSData data;
    public:
      static Barometer barometer;
      LocalMSData getLocalData();
      static void begin(TwoWire& wire, DetectorsCallback callback);
};