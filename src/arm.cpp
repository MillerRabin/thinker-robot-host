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
    Serial.printf("roll: %f, pitch: %f, yaw: %f\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Arm::begin(TwoWire& wire) {
  powerManagement.begin();
  powerManagement.enableEngines();
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

bool Arm::getFloat(JsonObject& jsonObj, const char* key, float& result) {
  if (!jsonObj.containsKey(key))
    return false;
  const char *str = jsonObj[key];
  result = atof(str);
  return true;
}



void Arm::set(JsonObject& data) {
  float shoulderY = NAN;
  bool hasShoulderY = getFloat(data, "shoulder-y", shoulderY);
  float shoulderZ = NAN;
  bool hasShoulderZ = getFloat(data, "shoulder-z", shoulderZ);
  uint8_t shoulderData[8];
  memcpy(shoulderData, &shoulderY, 4);
  memcpy(&shoulderData[4], &shoulderZ, 4);
  if (!twai.sendData(CAN_SHOULDER_SET_YZ_DEGREE, shoulderData)) {
    printf("Error sending data\n");
  }
}