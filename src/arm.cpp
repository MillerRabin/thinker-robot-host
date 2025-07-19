#include "arm.h"
#include "structures.h"

TWAI Arm::twai;
ArmShoulder Arm::shoulder;
ArmElbow Arm::elbow;
ArmWrist Arm::wrist;
ArmPlatform Arm::platform;
ArmStatus Arm::status;

PowerManagement Arm::powerManagement;

void Arm::twaiCallback(CanFrame frame) {
  uint32_t ident = frame.identifier;    
  if (ident == CAN_SHOULDER_QUATERNION) {
    Arm::status.shoulderQuaternionOK = true;
    shoulder.imu.quaternion.deserialize(frame.data);
    return;
  }    
  if (ident == CAN_SHOULDER_ACCELEROMETER) {
    shoulder.imu.accelerometer.deserialize(frame.data);
    return;
  }    
  if (ident == CAN_SHOULDER_GYROSCOPE) {
    shoulder.imu.gyroscope.deserialize(frame.data);
    return;
  }    
  if (ident == CAN_SHOULDER_ACCURACY) {
    shoulder.imu.accuracy.deserialize(frame.data);
    return;
  }
  if (ident == CAN_SHOULDER_STATUSES) {
    memcpy(&Arm::status.shoulderStatuses, frame.data, 8);
    return;
  }

  if (ident == CAN_ELBOW_QUATERNION) {
    Arm::status.elbowQuaternionOK = true;
    elbow.imu.quaternion.deserialize(frame.data);
    return;
  }
  if (ident == CAN_ELBOW_ACCELEROMETER) {
    elbow.imu.accelerometer.deserialize(frame.data);  
    return;
  }    
  if (ident == CAN_ELBOW_GYROSCOPE) {
    elbow.imu.gyroscope.deserialize(frame.data);  
    return;
  }    
  if (ident == CAN_ELBOW_ACCURACY) {
    elbow.imu.accuracy.deserialize(frame.data);  
    return;
  }

  if (ident == CAN_WRIST_QUATERNION) { 
    Arm::status.wristQuaternionOK = true;
    wrist.imu.quaternion.deserialize(frame.data);
    return;
  }
  if (ident == CAN_WRIST_ACCELEROMETER) {
    wrist.imu.accelerometer.deserialize(frame.data);  
    return;
  }

  if (ident == CAN_WRIST_GYROSCOPE) {
    wrist.imu.gyroscope.deserialize(frame.data);  
    return;
  }    

  if (ident == CAN_CLAW_QUATERNION) { 
    Arm::status.clawQuaternionOK = true;
    claw.imu.quaternion.deserialize(frame.data);
    return;
  }

  if (ident == CAN_CLAW_RANGE) {
    Arm::status.clawRangeOK = true;
    claw.range.deserialize(frame.data);
    return;
  }
}

void Arm::twaiErrorCallback(CanFrame frame, int code) {  
  if (code == CAN_SUCCESS) {
    Arm::status.canSendOK = true;
  } else {
    Arm::status.canSendOK = false;    
    printf("frame send error %d\n", frame.identifier);
  }  
}

void Arm::detectorsCallback(uint32_t id, uint64_t data) {
  if (!twai.sendData(id, (uint8_t*)&data)) {
    printf("Error sending detectors data\n");
  }
}

void Arm::loop(void* parameters) {
  while (true) {
    //Quaternion sQuat = shoulder.imu.quaternion;    
    //Euler sEuler = sQuat.getEuler();
    //uint8_t sAcc = shoulder.imu.accuracy.quaternionAccuracy;
            
    //printf("ShoulderBNO roll:  %f, pitch: %f, yaw: %f, accuracy: %d\n", sEuler.getRollAngle(), sEuler.getPitchAngle(), sEuler.getYawAngle(), sAcc);
    //Euler pEuler = platform.imu.quaternion.getEuler();
    //uint8_t pAcc = platform.imu.accuracy.quaternionAccuracy;  
    //printf("PlatformBNO roll: %f, pitch: %f, yaw: %f, accuracy: %d\n", pEuler.getRollAngle(), pEuler.getPitchAngle(), pEuler.getYawAngle(), pAcc);
    Arm::status.shoulderQuaternionOK = false;
    Arm::status.elbowQuaternionOK = false;
    Arm::status.wristQuaternionOK = false;
    Arm::status.clawRangeOK = false;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Arm::begin(TwoWire& wire, SPIClass& spi) {
  powerManagement.begin();      
  //powerManagement.enableCamera();
  //powerManagement.enableEngines();  
  twai.begin(twaiCallback, twaiErrorCallback);
  platform.begin(wire, spi, detectorsCallback); 
  xTaskCreate(
    loop,
    "Arm::loop",
    4096,
    NULL,
    5,
    NULL
  );
}

bool Arm::getFloat(JsonObject jsonObj, const char *key, float &result) {
  if (!jsonObj.containsKey(key))
    return false;

  JsonVariant value = jsonObj[key];
  if (value.is<float>() || value.is<long>() || value.is<double>()) {
    result = value.as<float>();
    return true;
  }

  if (value.is<const char *>()) {
    const char *str = value.as<const char *>();
    if (str) {
      result = atof(str);
      return true;
    }
  }

  return false;
}

bool Arm::getBool(JsonObject jsonObj, const char *key, bool &result)
{
  if (!jsonObj.containsKey(key))
    return false;

  JsonVariant value = jsonObj[key];

  if (value.is<bool>()) {
    result = value.as<bool>();
    return true;
  }

  
  if (value.is<int>() || value.is<unsigned int>() || value.is<long>() || value.is<unsigned long>()) {
    int intval = value.as<int>();
    result = (intval != 0);
    return true;
  }

  
  if (value.is<const char *>()) {
    const char *str = value.as<const char *>();
    if (str == nullptr)
      return false;

    if (strcasecmp(str, "true") == 0 || strcmp(str, "1") == 0) {
      result = true;
      return true;
    }

    if (strcasecmp(str, "false") == 0 || strcmp(str, "0") == 0) {
      result = false;
      return true;
    }
  }

  return false;
}

void Arm::set(JsonObject data) {  
  bool enginesEnabled = false;
  bool hasEngines = getBool(data, "enginesEnabled", enginesEnabled);  
  bool cameraEnabled = false;
  bool hasCamera = getBool(data, "cameraEnabled", cameraEnabled);
  if (hasEngines)
    enginesEnabled ? powerManagement.enableEngines() : powerManagement.disableEngines();

  if (hasCamera)
    cameraEnabled ? powerManagement.enableCamera() : powerManagement.disableCamera();
  
  float shoulderY = NAN;    
  bool hasShoulderY = getFloat(data, "shoulder-y", shoulderY);
  float shoulderZ = NAN;
  bool hasShoulderZ = getFloat(data, "shoulder-z", shoulderZ);
  if (hasShoulderY || hasShoulderZ) {
    uint32_t encoded = 0;
    if (hasShoulderY)
      encoded |= (uint16_t)(shoulderY * 100) & 0xFFFF;
    if (hasShoulderZ)
      encoded |= ((uint32_t)(shoulderZ * 100) & 0xFFFF) << 16;

    uint8_t shoulderData[8];
    memcpy(shoulderData, &encoded, 4);
    if (!twai.sendData(CAN_SHOULDER_SET_YZ_DEGREE, shoulderData)) {
      printf("Error sending data\n");
    }
  }
}

void Arm::setRotate(JsonObject data) {
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

StatusResponse Arm::upgrade(JsonObject data) {      
  uint8_t sData[8] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
  if (!data.containsKey("part")) {
   return StatusResponse({ "No key of part found", RESPONSE_STATUS_NO_ARM_PART_KEY_FOUND });
  }    
  const char *b = data["part"];
  std::string str = std::string(b);  
  if (str == "shoulder") {
    if (!twai.sendData(CAN_SHOULDER_FIRMWARE_UPGRADE, sData)) {      
      return StatusResponse({ "Shoulder TWAI upgrade command error", RESPONSE_STATUS_TWAI_SEND_DATA_ERROR });
    }
  }
  if (str == "elbow") {
    if (!twai.sendData(CAN_ELBOW_FIRMWARE_UPGRADE, sData)) {      
      return StatusResponse({ "elbow TWAI upgrade command error", RESPONSE_STATUS_TWAI_SEND_DATA_ERROR });
    }
  }
  if (str == "wrist") {
    if (!twai.sendData(CAN_WRIST_FIRMWARE_UPGRADE, sData)) {      
      return StatusResponse({ "Wrist TWAI upgrade command error", RESPONSE_STATUS_TWAI_SEND_DATA_ERROR });
    }
  }
  if (str == "claw") {      
    if (!twai.sendData(CAN_CLAW_FIRMWARE_UPGRADE, sData)) {      
      return StatusResponse({ "Claw TWAI upgrade command error", RESPONSE_STATUS_TWAI_SEND_DATA_ERROR });
    }
  }    
  return StatusResponse({ "Upgrade success", RESPONSE_STATUS_SUCCESS });;
}