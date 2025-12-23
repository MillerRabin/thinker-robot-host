#include "accelerometer.h"

uint64_t Accelerometer::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Accelerometer::deserialize(uint8_t data[8]) {
  this->x = (int16_t)data[1] << 8 | data[0];
  this->y = (int16_t)data[3] << 8 | data[2];
  this->z = (int16_t)data[5] << 8 | data[4];  
}

void Accelerometer::set(int16_t rawAccX, int16_t rawAccY, int16_t rawAccZ) {
  x = rawAccX;
  y = rawAccY;
  z = rawAccZ;
}