#include "arm.h"
#include "structures.h"

TWAI Arm::twai;
ArmShoulder Arm::shoulder;
ArmPlatform Arm::platform;
PowerManagement Arm::powerManagement;


void Arm::twaiCallback(CanFrame frame) {
  uint32_t ident = frame.identifier;  
  if (ident == CAN_SHOULDER_QUATERNION)
    shoulder.imu.quaternion.deserialize(frame.data);  
}

void Arm::loop(void* parameters) {
  while (true) {
    Euler sEuler = shoulder.imu.quaternion.getEuler();
    float roll = sEuler.roll * 180.0 / PI; // Convert roll to degrees
    float pitch = sEuler.pitch * 180.0 / PI; // Convert pitch to degrees
    float yaw = sEuler.yaw * 180.0 / PI; // Convert yaw / heading to degrees
    Serial.printf(" roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Arm::begin(TwoWire& wire) {
  powerManagement.begin();
  twai.begin(twaiCallback);
  platform.begin(wire);
  xTaskCreate(
    loop,
    "Arm::loop",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    NULL
  );
}