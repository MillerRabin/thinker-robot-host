#include "arm.h"

TWAI Arm::twai;
ArmShoulder Arm::shoulder;
ArmPlatform Arm::platform;
PowerManagement Arm::powerManagement;

void Arm::getQuaternion(uint8_t data[8]) {

}

void Arm::twaiCallback(CanFrame frame) {
  uint32_t ident = frame.identifier;
  if (ident == CAN_SHOULDER_QUATERNION)
    getQuaternion(frame.data);
  //Serial.printf("Received 0x%x\n", frame.identifier);
}

void Arm::begin(TwoWire& wire) {
  powerManagement.begin();
  twai.begin(twaiCallback);
  platform.begin(wire);
  shoulder.begin();
}