#pragma once

#include <Arduino.h>
#include "qBase.h"
#include "euler.h"

class Quaternion;

class Quaternion : public QBase {
public:
  float i;
  float j;
  float k;
  float real;
  uint8_t Q1 = 14;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool fromBNO(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
  bool fromWitmotion(int16_t rawQuatI, int16_t rawQuatJ, int16_t rawQuatK, int16_t rawQuatReal);
  Quaternion(float i = 0, float j = 0, float k = 0, float real = 1.0f) : i(i), j(j), k(k), real(real) {}  
  Quaternion operator*(const Quaternion &q) const {
    return Multiply(*this, q);    
  }
  Euler toEuler();  
  static Quaternion FromEuler(float roll, float pitch, float yaw);  
  static Quaternion Conjugate(const Quaternion &q);
  static Quaternion Multiply(const Quaternion &a, const Quaternion &b);
};