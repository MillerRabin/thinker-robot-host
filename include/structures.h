
#pragma once

#include <Arduino.h>

typedef void (* DetectorsCallback)(uint32_t id, uint64_t data);

class Euler {
  public:
    Euler(const float roll, const float pitch, const float yaw);
    const float yaw;
    const float pitch;
    const float roll;
    const float getRollAngle();
    const float getPitchAngle();
    const float getYawAngle();
};

class StructureBasic {
  public:
    float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
};

class Quaternion : public StructureBasic {
private:
  void convertRawData();
public:
  uint16_t rawI;
  uint16_t rawJ;
  uint16_t rawK;
  uint16_t rawReal;
  float i;
  float j;
  float k;
  float real;
  uint8_t Q1 = 14;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();  
  void deserialize(uint8_t data[8]);
  Euler getEuler();
  bool set(uint16_t rawQuatI, uint16_t rawQuatJ, uint16_t rawQuatK, uint16_t rawQuatReal);
};

class Accuracy {
public:
  uint16_t quaternionRadAccuracy;
  uint8_t quaternionAccuracy;
  uint8_t accelerometerAccuracy;
  uint8_t gyroscopeAccuracy;  
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t quaternionRadianAccuracy, uint8_t quaternionAccuracy, uint8_t gyroscopeAccuracy, uint8_t accelerometerAccuracy);
};

class Accelerometer {
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t rawAccX, uint16_t rawAccY, uint16_t rawAccZ);
};

class Gyroscope {
public:
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t Q1;
  uint8_t Q2;
  uint8_t Q3;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(uint16_t rawGyroX, uint16_t rawGyroY, uint16_t rawGyroZ);
};

class Barometer {
public:
  float pressure;
  float temperature;
  uint64_t serialize();
  void deserialize(uint8_t data[8]);
  bool set(float pressure, float temperature);
};
