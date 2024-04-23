#pragma once

#include <Wire.h>
#include <MS5611.h>

class PMS5611 {
    private:
      static MS5611 ms;
      static void loop(void* parameters);
    public:
      static volatile float pressure;
      static volatile float temperature;
      static void begin(TwoWire& wire);
};