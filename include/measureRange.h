#pragma once

#include <Arduino.h>

class MeasureRange {
private:
  uint16_t longRange;
  uint16_t shortRange;

public:
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  void set(uint16_t longRange, uint16_t shortRange);
};