#include "quaternion.h"

uint64_t IMUQuaternion::serialize() {
  return (uint64_t)this->rawI |
         (uint64_t)this->rawJ << 16 |
         (uint64_t)this->rawK << 32 |
         (uint64_t)this->rawReal << 48;
}

void IMUQuaternion::deserialize(uint8_t data[8]) {
  this->rawI = (uint16_t)data[1] << 8 | data[0];
  this->rawJ = (uint16_t)data[3] << 8 | data[2];
  this->rawK = (uint16_t)data[5] << 8 | data[4];
  this->rawReal = (uint16_t)data[7] << 8 | data[6];
}

void IMUQuaternion::convertRawData() {
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
  //applyRotate();
}

void IMUQuaternion::convertRawDataByDivision(float divisor) {
  this->i = rawI / divisor;
  this->j = rawJ / divisor;
  this->k = rawK / divisor;
  this->real = rawReal / divisor;
}

bool IMUQuaternion::set(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal) {
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

Euler IMUQuaternion::getEuler() {
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

Quaternion::Quaternion(IMUQuaternion &q) {  
  q.convertRawData();
  i = q.i;
  j = q.j;
  k = q.k;
  real = q.real;
}

Quaternion Quaternion::FromEuler(float roll, float pitch, float yaw){
  float cr = cos(roll / 2.0f);
  float sr = sin(roll / 2.0f);
  float cp = cos(pitch / 2.0f);
  float sp = sin(pitch / 2.0f);
  float cy = cos(yaw / 2.0f);
  float sy = sin(yaw / 2.0f);

  Quaternion q;
  q.real = cr * cp * cy + sr * sp * sy;
  q.i = sr * cp * cy - cr * sp * sy;
  q.j = cr * sp * cy + sr * cp * sy;
  q.k = cr * cp * sy - sr * sp * cy;

  return q;
}

Quaternion Quaternion::Conjugate(const Quaternion &q)
{
  return {-q.i, -q.j, -q.k, q.real};
}

Quaternion Quaternion::Multiply(const Quaternion &a, const Quaternion &b)
{
  Quaternion q;
  q.real = a.real * b.real - a.i * b.i - a.j * b.j - a.k * b.k;
  q.i = a.real * b.i + a.i * b.real + a.j * b.k - a.k * b.j;
  q.j = a.real * b.j - a.i * b.k + a.j * b.real + a.k * b.i;
  q.k = a.real * b.k + a.i * b.j - a.j * b.i + a.k * b.real;
  return q;
}
