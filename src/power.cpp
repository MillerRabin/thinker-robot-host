#include "power.h"

uint32_t Power::serialize() {
  uint32_t result = 0;
  result |= voltage << 16;
  result |= current;
  return result;
}

void Power::deserialize(uint8_t data[4]) {
  voltage = (data[0] << 8) | data[1];
  current = (data[2] << 8) | data[3];
}