#include "barometer.h"

uint64_t Barometer::serialize() {
  return (uint64_t)temperatureRaw << 56 | (uint64_t)pressureRaw << 32 | (uint64_t)heightRaw;
  }

void Barometer::deserialize(uint8_t data[8])
{
  memcpy(&heightRaw, data, 4);
  pressureRaw = 0;
  memcpy(&pressureRaw, &data[4], 3);
  memcpy(&temperatureRaw, &data[8], 1);
}

void Barometer::set(uint16_t height, uint16_t pressure, uint8_t temperature) {  
  this->temperatureRaw = temperature;
  this->pressureRaw = pressure;
  this->heightRaw = height;
}
