#include "structures.h"


Euler::Euler(const float roll, const float pitch, const float yaw) : 
  roll(roll),
  pitch(pitch),
  yaw(yaw)
{}

const float Euler::getRollAngle() { return roll * 180.0 / PI; }
const float Euler::getPitchAngle() { return pitch * 180.0 / PI; }
const float Euler::getYawAngle() { return yaw * 180.0 / PI; }

float StructureBasic::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return qFloat;
}

uint64_t Quaternion::serialize() {
  return (uint64_t)this->rawI |
         (uint64_t)this->rawJ << 16 |
         (uint64_t)this->rawK << 32 |
         (uint64_t)this->rawReal << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  this->rawI = (uint16_t)data[1] << 8 | data[0];
  this->rawJ = (uint16_t)data[3] << 8 | data[2];
  this->rawK = (uint16_t)data[5] << 8 | data[4];
  this->rawReal = (uint16_t)data[7] << 8 | data[6];
}

void Quaternion::convertRawData() {
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
}

bool Quaternion::set(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
  uint16_t id = (rawQuatI > rawI) ? rawQuatI - rawI : rawI - rawQuatI;
  uint16_t jd = (rawQuatJ > rawJ) ? rawQuatJ - rawJ : rawJ - rawQuatJ;
  uint16_t kd = (rawQuatK > rawK) ? rawQuatK - rawK : rawK - rawQuatK;
  uint16_t rd = (rawQuatReal >rawReal) ? rawQuatReal - rawReal : rawReal - rawQuatReal;
  this->rawI = rawQuatI;
  this->rawJ = rawQuatJ;
  this->rawK = rawQuatK;
  this->rawReal = rawQuatReal;
  return ((id > 1) || (jd > 1) || (kd > 1) || (rd > 1));
}


Euler Quaternion::getEuler() {
  convertRawData();  
  float dqw = real;
	float dqx = i;
	float dqy = j;
	float dqz = k;

	float norm = sqrt(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
	dqw = dqw/norm;
	dqx = dqx/norm;
	dqy = dqy/norm;
	dqz = dqz/norm;

	float ysqr = dqy * dqy;
	
	float t3 = +2.0 * (dqw * dqz + dqx * dqy);
	float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
	float yaw = atan2(t3, t4);

	float t2 = +2.0 * (dqw * dqy - dqz * dqx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float pitch = asin(t2);
  	
	float t0 = +2.0 * (dqw * dqx + dqy * dqz);
	float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
	float roll = atan2(t0, t1);
	  
  return Euler(roll, pitch, yaw);
}

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
  this->x = (uint16_t)data[1] << 8 | data[0];
  this->y = (uint16_t)data[3] << 8 | data[2];
  this->z = (uint16_t)data[5] << 8 | data[4];  
}

bool Gyroscope::set(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ) {
  uint16_t xd = (rawGyroX > x) ? rawGyroX - x : x - rawGyroX;
  uint16_t yd = (rawGyroY > y) ? rawGyroY - y : y - rawGyroY;
  uint16_t zd = (rawGyroZ > z) ? rawGyroZ - z : z - rawGyroZ;  
  x = rawGyroX;
  y = rawGyroY;
  z = rawGyroZ;  
  return ((xd > 1) || (yd > 1) || (zd > 1));
}

bool Accuracy::set(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy) {
  bool rd = (quaternionRadAccuracy != quaternionRadianAccuracy);
  bool qd = (quaternionAccuracy != quaternionAccuracy);
  bool gd = (gyroscopeAccuracy != gyroscopeAccuracy);
  bool ad = (accelerometerAccuracy != accelerometerAccuracy);
  accelerometerAccuracy = accelerometerAccuracy;
  quaternionAccuracy = quaternionAccuracy;
  quaternionRadAccuracy = quaternionRadianAccuracy;  
  gyroscopeAccuracy = gyroscopeAccuracy;
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

uint64_t Barometer::serialize() {
  uint8_t res[8];
  memcpy(res, &pressure, 4);
  memcpy(&res[4], &temperature, 4);
  return (uint64_t)res;
}

void Barometer::deserialize(uint8_t data[8]) {
  memcpy(&pressure, data, 4);
  memcpy(&temperature, &data[4], 4);  
}

bool Barometer::set(float pressure, float temperature) {
  float pd = (this->pressure > pressure) ? this->pressure - pressure : pressure - this->pressure;
  float td = (this->temperature > temperature) ? this->temperature - temperature : temperature - this->temperature;  
  this->temperature = temperature;
  this->pressure = pressure;
  return (pd > 0.1) || (td > 0.1);
}
