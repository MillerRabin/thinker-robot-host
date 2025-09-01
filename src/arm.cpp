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

  if (ident == CAN_ELBOW_STATUSES) {
    memcpy(&Arm::status.elbowStatuses, frame.data, 8);
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

  if (ident == CAN_WRIST_STATUSES) {
    memcpy(&Arm::status.wristStatuses, frame.data, 8);
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

  if (ident == CAN_CLAW_STATUSES) {
    memcpy(&Arm::status.clawStatuses, frame.data, 8);
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
    Arm::status.shoulderQuaternionOK = false;
    Arm::status.elbowQuaternionOK = false;
    Arm::status.wristQuaternionOK = false;
    Arm::status.clawRangeOK = false;
    Arm::status.clawQuaternionOK = false;
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
  if (key == NULL) {
    return false;
  }

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

void Arm::setPowerState(JsonObject data) {
  bool enginesEnabled = false;
  bool hasEngines = getBool(data, "enginesEnabled", enginesEnabled);
  bool cameraEnabled = false;
  bool hasCamera = getBool(data, "cameraEnabled", cameraEnabled);
  if (hasEngines)
    enginesEnabled ? powerManagement.enableEngines() : powerManagement.disableEngines();

  if (hasCamera)
    cameraEnabled ? powerManagement.enableCamera() : powerManagement.disableCamera();
}
bool Arm::sendArmData(
    JsonObject data,
    const char *key1,
    const char *key2,
    const char *key3,
    const char *key4,
    const uint32_t canMessage) {
  float val1 = NAN;
  bool hasVal1 = getFloat(data, key1, val1);
  float val2 = NAN;
  bool hasVal2 = getFloat(data, key2, val2);
  float val3 = NAN;
  bool hasVal3 = getFloat(data, key3, val3);
  float val4 = NAN;
  bool hasVal4 = getFloat(data, key4, val4);
  
  auto isValid = [](float v) {
    return v >= 0.0f && v <= 320.0f;
  };


  if ((hasVal1 && !isValid(val1)) ||
      (hasVal2 && !isValid(val2)) ||
      (hasVal3 && !isValid(val3)) ||
      (hasVal4 && !isValid(val4))) {    
    return false;
  }

  if (hasVal1 || hasVal2 || hasVal3 || hasVal4) {
    ArmDataFrame frame;
    frame.values.param1 = hasVal1 ? (int16_t)(val1 * 100) : PARAMETER_IS_NAN;
    frame.values.param2 = hasVal2 ? (int16_t)(val2 * 100) : PARAMETER_IS_NAN;
    frame.values.param3 = hasVal3 ? (int16_t)(val3 * 100) : PARAMETER_IS_NAN;
    frame.values.param4 = hasVal4 ? (int16_t)(val4 * 100) : PARAMETER_IS_NAN;
    return twai.sendData(canMessage, frame.bytes);
  }
  
  return false;
}

void Arm::set(JsonObject data) {
  setPowerState(data);

  struct {
    const char *param1;
    const char *param2;
    const char *param3;
    const char *param4;
    const uint32_t canId;
    const char *name;
  } actions[] = {
      {"shoulder-y", "shoulder-z", "time", nullptr, CAN_SHOULDER_SET_YZ_DEGREE, "shoulder"},
      {"elbow-y", "time", nullptr, nullptr, CAN_ELBOW_SET_Y_DEGREE, "elbow"},
      {"wrist-y", "wrist-z", "time", nullptr, CAN_WRIST_SET_YZ_DEGREE, "wrist"},
      {"claw-y", "claw-x", "claw-gripper", "time", CAN_CLAW_SET_XYG_DEGREE, "claw"}};

  for (auto &action : actions) {
    sendArmData(data, action.param1, action.param2, action.param3, action.param4, action.canId);
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