#include "range.h"

uint64_t Range::serialize() {
  return (uint64_t)range |
         (uint64_t)measureType << 16;
}

void Range::deserialize(uint8_t data[8]) {
  this->range = (uint32_t)data;
  this->measureType = (uint32_t)&data[4];
}

bool Range::set(uint32_t range, uint8_t measureType) {
  this->range = range;
  this->measureType = measureType;
}