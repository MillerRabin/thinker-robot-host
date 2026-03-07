#include "quaternion.h"

Quaternion Quaternion::FromEuler(float roll, float pitch, float yaw) {
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

Quaternion Quaternion::Conjugate(const Quaternion &q) {
  return {-q.i, -q.j, -q.k, q.real};
}

Quaternion Quaternion::Multiply(const Quaternion &a, const Quaternion &b) {
  Quaternion q;
  q.real = a.real * b.real - a.i * b.i - a.j * b.j - a.k * b.k;
  q.i = a.real * b.i + a.i * b.real + a.j * b.k - a.k * b.j;
  q.j = a.real * b.j - a.i * b.k + a.j * b.real + a.k * b.i;
  q.k = a.real * b.k + a.i * b.j - a.j * b.i + a.k * b.real;
  return q;
}

uint64_t Quaternion::serialize() {
  uint64_t rawI = QBase::floatToQ(i, Q1);
  uint64_t rawJ = QBase::floatToQ(j, Q1);
  uint64_t rawK = QBase::floatToQ(k, Q1);
  uint64_t rawReal = QBase::floatToQ(real, Q1);
  return (uint64_t)rawI | (uint64_t)rawJ << 16 | (uint64_t)rawK << 32 |
         (uint64_t)rawReal << 48;
}

void Quaternion::deserialize(uint8_t data[8]) {
  uint16_t rawI = (uint16_t)data[1] << 8 | data[0];
  uint16_t rawJ = (uint16_t)data[3] << 8 | data[2];
  uint16_t rawK = (uint16_t)data[5] << 8 | data[4];
  uint16_t rawReal = (uint16_t)data[7] << 8 | data[6];
  this->i = qToFloat(rawI, Q1);
  this->j = qToFloat(rawJ, Q1);
  this->k = qToFloat(rawK, Q1);
  this->real = qToFloat(rawReal, Q1);
}

bool Quaternion::fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ,
                         uint16_t rawQuatK, uint16_t rawQuatReal) {
  this->i = qToFloat(rawQuatI, Q1);
  this->j = qToFloat(rawQuatJ, Q1);
  this->k = qToFloat(rawQuatK, Q1);
  this->real = qToFloat(rawQuatReal, Q1);
  return true;
}

bool Quaternion::fromWitmotion(int16_t rawQuatI, int16_t rawQuatJ,
                               int16_t rawQuatK, int16_t rawQuatReal) {
  this->i = rawQuatI / 32768.0f;
  this->j = rawQuatJ / 32768.0f;
  this->k = rawQuatK / 32768.0f;
  this->real = rawQuatReal / 32768.0f;

  float norm = sqrtf(this->real * this->real + this->i * this->i +
                     this->j * this->j + this->k * this->k);    
  if (norm > 0.0001f) {
    this->real = real / norm;
    this->i = i / norm;
    this->j = j / norm;
    this->k = k / norm;
  }

  return true;
}

Euler Quaternion::toEuler() {
  float dqw = real;
  float dqx = i;
  float dqy = j;
  float dqz = k;

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  dqw = dqw / norm;
  dqx = dqx / norm;
  dqy = dqy / norm;
  dqz = dqz / norm;

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