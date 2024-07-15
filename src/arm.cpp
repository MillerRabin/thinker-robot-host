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
  if (ident == CAN_SHOULDER_ACCELEROMETER)
    shoulder.imu.accelerometer.deserialize(frame.data);  
  if (ident == CAN_SHOULDER_GYROSCOPE)
    shoulder.imu.gyroscope.deserialize(frame.data);  
  if (ident == CAN_SHOULDER_ACCURACY)
    shoulder.imu.accuracy.deserialize(frame.data);  
}

void Arm::detectorsCallback(uint32_t id, uint64_t data) {
  if (!twai.sendData(id, (uint8_t*)&data)) {
    printf("Error sending detectors data\n");
  }
}

void Arm::loop(void* parameters) {
  while (true) {
    Quaternion sQuat = shoulder.imu.quaternion;    
    Euler sEuler = sQuat.getEuler();
    uint8_t sAcc = shoulder.imu.accuracy.quaternionAccuracy;
            
    printf("ShoulderBNO roll:  %f, pitch: %f, yaw: %f, accuracy: %d\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle(), sAcc);
    Euler pEuler = platform.imu.quaternion.getEuler();
    uint8_t pAcc = platform.imu.accuracy.quaternionAccuracy;  
    printf("PlatformBNO roll: %f, pitch: %f, yaw: %f, accuracy: %d\n", pEuler.getRollAngle(), pEuler.getPitchAngle(), pEuler.getYawAngle(), pAcc);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Arm::begin(TwoWire& wire, SPIClass& spi) {
  powerManagement.begin();    
  //shoulder.imu.quaternion.setRotate(0.0F, 0.7071F, 0.0F, 0.7071F);
  //powerManagement.enableCamera();
  powerManagement.enableEngines();
  twai.begin(twaiCallback);
  platform.begin(wire, spi, detectorsCallback);
  /*xTaskCreate(
    loop,
    "Arm::loop",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    NULL
  );*/
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

void Arm::setRotate(JsonObject& data) {
  float shoulderReal = 0;
  getFloat(data, "shoulder-real", shoulderReal);
  float shoulderI = 0;
  getFloat(data, "shoulder-i", shoulderI);
  float shoulderJ = 0;
  getFloat(data, "shoulder-j", shoulderJ);
  float shoulderK = 0;
  getFloat(data, "shoulder-k", shoulderK);
  /*shoulder.imu.quaternion.setRotate(shoulderI, shoulderJ, shoulderK, shoulderReal);  
  uint64_t qData = shoulderRotate.serialize();
  uint8_t sData[8];
  memcpy(sData, &qData, sizeof(uint64_t));
  
  if (!twai.sendData(CAN_SHOULDER_SET_ROTATE_RADIAN, sData)) {
    printf("Error sending rotate data\n");
  }*/
}