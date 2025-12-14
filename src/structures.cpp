#include "structures.h"

uint64_t Accelerometer::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Accelerometer::deserialize(uint8_t data[8]) {
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}

bool Accelerometer::set(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ) {
  uint16_t xd = (rawAccX > x) ? rawAccX - x : x - rawAccX;
  uint16_t yd = (rawAccY > y) ? rawAccY - y : y - rawAccY;
  uint16_t zd = (rawAccZ > z) ? rawAccZ - z : z - rawAccZ;  
  x = rawAccX;
  y = rawAccY;
  z = rawAccZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}


uint64_t Gyroscope::serialize() {
  return (uint64_t)this->x |
         (uint64_t)this->y << 16 |
         (uint64_t)this->z << 32;
}

void Gyroscope::deserialize(uint8_t data[8]) {
  x = (uint16_t)data[1] << 8 | data[0];
  y = (uint16_t)data[3] << 8 | data[2];
  z = (uint16_t)data[5] << 8 | data[4];  
}

bool Gyroscope::set(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > x) ? rawGyroX - x : x - rawGyroX;
  uint16_t yd = (rawGyroY > y) ? rawGyroY - y : y - rawGyroY;
  uint16_t zd = (rawGyroZ > z) ? rawGyroZ - z : z - rawGyroZ;  
  this->x = rawGyroX;
  this->y = rawGyroY;
  this->z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

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