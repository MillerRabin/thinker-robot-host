#pragma once

#include <Arduino.h>

class Range {    
  public:
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    void set(uint32_t range, uint8_t measureType);
    uint16_t range;
    uint8_t measureType;;
};
