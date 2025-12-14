#pragma once

#include <Wire.h>
#include <MS5611.h>
#include "structures.h"
#include "barometer.h"
#include "config.h"

class PMS5611 {
    private:
      static MS5611 ms;
      static void loop(void* parameters);      
      static DetectorsCallback callback;
    public:
      static Barometer barometer;
      static void begin(TwoWire& wire, DetectorsCallback callback);
};