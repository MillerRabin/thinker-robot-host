#include "accuracy.h"

bool Accuracy::set(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy) {
  bool rd = (quaternionRadAccuracy != quaternionRadianAccuracy);
  bool qd = (quaternionAccuracy != quaternionAccuracy);
  bool gd = (gyroscopeAccuracy != gyroscopeAccuracy);
  bool ad = (accelerometerAccuracy != accelerometerAccuracy);
  this->accelerometerAccuracy = accelerometerAccuracy;
  this->quaternionAccuracy = quaternionAccuracy;
  this->quaternionRadAccuracy = quaternionRadianAccuracy;  
  this->gyroscopeAccuracy = gyroscopeAccuracy;
  return ad || rd || qd || gd;
}

uint64_t Accuracy::serialize() {
  return (uint64_t)this->quaternionAccuracy |
         (uint64_t)this->accelerometerAccuracy << 8 |
         (uint64_t)this->quaternionRadAccuracy << 16 |
         (uint64_t)this->gyroscopeAccuracy << 32;
}

void Accuracy::deserialize(uint8_t data[8]) {
  this->quaternionAccuracy = data[0];
  this->accelerometerAccuracy = data[1];
  this->quaternionRadAccuracy = (uint16_t)data[3] << 8 | data[2];
  this->gyroscopeAccuracy = data[4];  
}