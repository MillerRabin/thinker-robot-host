#include "range.h"

uint64_t Range::serialize() {
  return (uint64_t)range |
         (uint64_t)measureType << 16;
}

void Range::deserialize(uint8_t data[8]) {    
  this->range = (uint16_t)data[1] << 8 | data[0];
  this->measureType = (uint16_t)data[3] << 8 | data[2];
}

void Range::set(uint32_t range, uint8_t measureType) {
  this->range = range;
  this->measureType = measureType;
}