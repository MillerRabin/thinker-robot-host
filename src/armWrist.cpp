#include "armWrist.h"

RemoteBNO ArmWrist::imu;

namespace {
portMUX_TYPE anglesLock = portMUX_INITIALIZER_UNLOCKED;
float pitch = NAN;
float bend = NAN;
uint32_t heardAt = 0;
}  // namespace

void ArmWrist::setAngles(const uint8_t data[8]) {
  float p, b;
  memcpy(&p, data, sizeof(float));
  memcpy(&b, data + 4, sizeof(float));
  portENTER_CRITICAL(&anglesLock);
  pitch = p;
  bend = b;
  heardAt = millis() | 1;
  portEXIT_CRITICAL(&anglesLock);
}

void ArmWrist::readAngles(float &pitchDeg, float &bendDeg, uint32_t &at) {
  portENTER_CRITICAL(&anglesLock);
  pitchDeg = pitch;
  bendDeg = bend;
  at = heardAt;
  portEXIT_CRITICAL(&anglesLock);
}
