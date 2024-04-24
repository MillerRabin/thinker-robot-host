#pragma once

#include <Wire.h>
#include <MS5611.h>
#include "structures.h"
#include "config.h"

class PMS5611 {
    private:
      static MS5611 ms;
      static void loop(void* parameters);
      static Barometer barometer;
      static DetectorsCallback callback;
    public:      
      static void begin(TwoWire& wire, DetectorsCallback callback);
};