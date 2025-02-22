#pragma once

#include <Arduino.h>

class Range {
  private:
    uint32_t range;
    uint8_t measureType;;
  public:
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    bool set(uint32_t range, uint8_t measureType);
};
