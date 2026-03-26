#include "arm.h"

TWAI Arm::twai((gpio_num_t)TWAI_RX_GPIO, (gpio_num_t)TWAI_TX_GPIO, Arm::twaiCallback, Arm::twaiErrorCallback);
ArmShoulder Arm::shoulder;
ArmElbow Arm::elbow;
ArmWrist Arm::wrist;
ArmPlatform Arm::platform(detectorsCallback);
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

  if (ident == CAN_CLAW_ACCELEROMETER) {
    claw.imu.accelerometer.deserialize(frame.data);    
    return;
  }

  if (ident == CAN_CLAW_GYROSCOPE) {
    claw.imu.gyroscope.deserialize(frame.data);
    return;
  }

  if (ident == CAN_CLAW_RANGE) {
    Arm::status.clawRangeOK = true;
    claw.range.deserialize(frame.data);
    return;
  }

  if (ident == CAN_CLAW_HEIGHT) {
    claw.barometer.deserialize(frame.data);
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
    //printf("frame send error 0x%x, code %d\n", frame.identifier, code);
  }  
}

void Arm::detectorsCallback(uint32_t id, uint64_t data) {
  
  twai.sendData(id, (uint8_t*)&data);  
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

void Arm::begin(TwoWire& wire) {
  powerManagement.begin();      
  twai.begin(TWAI_SPEED_1000KBPS);
  platform.begin(wire);
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
  bool cpuPowerEnabled = false;
  bool hasCPUPowerEnabled = getBool(data, "cpuPowerEnabled", cpuPowerEnabled);
  bool detectorsPowerDisabled = false;
  bool hasDetecorsPowerDisabled = getBool(data, "detectorsPowerDisabled", detectorsPowerDisabled);
    
  if (hasEngines)
    enginesEnabled ? powerManagement.enableEngines() : powerManagement.disableEngines();

  if (hasCamera)
    cameraEnabled ? powerManagement.enableCamera() : powerManagement.disableCamera();

  if (hasCPUPowerEnabled)
    cpuPowerEnabled ? powerManagement.enableCPUPower() : powerManagement.disableCPUPower();

  if (hasDetecorsPowerDisabled)    
    detectorsPowerDisabled ? powerManagement.disableDetectors() : powerManagement.enableDetectors(); 
}

bool Arm::sendArmData(JsonObject data, const char *key1,
                      int16_t (*parser1)(float), const char *key2,
                      int16_t (*parser2)(float), const char *key3,
                      int16_t (*parser3)(float), const char *key4,
                      int16_t (*parser4)(float), const uint32_t canMessage) {
  float val1 = NAN;
  bool hasVal1 = (key1 != nullptr && parser1 != nullptr) ? getFloat(data, key1, val1) : false;

  float val2 = NAN;
  bool hasVal2 = (key2 != nullptr && parser2 != nullptr) ? getFloat(data, key2, val2) : false;

  float val3 = NAN;
  bool hasVal3 = (key3 != nullptr && parser3 != nullptr) ? getFloat(data, key3, val3) : false;

  float val4 = NAN;
  bool hasVal4 = (key4 != nullptr && parser4 != nullptr) ? getFloat(data, key4, val4) : false;

  if (!(hasVal1 || hasVal2 || hasVal3 || hasVal4)) {
    return false;
  }

  ArmDataFrame frame{};
  frame.values.param1 = hasVal1 ? parser1(val1) : PARAMETER_IS_NAN;
  frame.values.param2 = hasVal2 ? parser2(val2) : PARAMETER_IS_NAN;
  frame.values.param3 = hasVal3 ? parser3(val3) : PARAMETER_IS_NAN;
  frame.values.param4 = hasVal4 ? parser4(val4) : PARAMETER_IS_NAN;

  return twai.sendData(canMessage, frame.bytes);
}

void Arm::set(JsonObject data) {
  setPowerState(data);

  auto getAngle = [](float v) -> int16_t {
    if (isnan(v) || v < 0.0f || v > 270.0f) {
      return PARAMETER_IS_NAN;
    }
    return static_cast<int16_t>(v * 10.0f);
  };

  auto getTime = [](float v) -> int16_t {
    if (isnan(v) || v < 0.0f || v > 65535.0f) {
      return PARAMETER_IS_NAN;
    }
    return static_cast<int16_t>(v);
  };

  struct Action {
    const char *param1;
    Parser parser1;
    const char *param2;
    Parser parser2;
    const char *param3;
    Parser parser3;
    const char *param4;
    Parser parser4;
    uint32_t canId;
    const char *name;
  };

  Action actions[] = {
      {"shoulder-y", getAngle, "shoulder-z", getAngle, "timeMS", getTime, nullptr, nullptr, CAN_SHOULDER_SET_YZ_DEGREE, "shoulder"},
      {"elbow-y", getAngle, "timeMS", getTime, nullptr, nullptr, nullptr, nullptr, CAN_ELBOW_SET_Y_DEGREE, "elbow"},
      {"wrist-y", getAngle, "wrist-z", getAngle, "timeMS", getTime, nullptr, nullptr, CAN_WRIST_SET_YZ_DEGREE, "wrist"},
      {"claw-y", getAngle, "claw-x", getAngle, "claw-gripper", getAngle, "timeMS", getTime, CAN_CLAW_SET_XYG_DEGREE, "claw"},
  };

  for (auto &action : actions) {
    sendArmData(data, action.param1, action.parser1, 
                      action.param2, action.parser2, 
                      action.param3, action.parser3, 
                      action.param4, action.parser4, 
                      action.canId);
  }
}

void Arm::tare(JsonObject data) {  
  bool platformTare = false;
  bool hasPlatformTare = getBool(data, "tare-platform", platformTare);
  bool platformClearTare = false;
  bool hasPlatformClearTare = getBool(data, "clear-platform", platformClearTare);
  bool shoulderTare = false;
  bool hasShoulderTare = getBool(data, "tare-shoulder", shoulderTare);
  bool shoulderClearTare = false;
  bool hasShoulderClearTare = getBool(data, "clear-shoulder", shoulderClearTare);
  bool elbowTare = false;
  bool hasElbowTare = getBool(data, "tare-elbow", elbowTare);
  bool elbowClearTare = false;
  bool hasElbowClearTare = getBool(data, "clear-elbow", elbowClearTare);
  bool wristTare = false;
  bool hasWristTare = getBool(data, "tare-wrist", wristTare);
  bool wristClearTare = false;
  bool hasWristClearTare = getBool(data, "clear-wrist", wristClearTare);
  bool clawTare = false;
  bool hasClawTare = getBool(data, "tare-claw", clawTare);
  bool clawClearTare = false;
  bool hasClawClearTare = getBool(data, "clear-claw", clawClearTare);

  uint16_t clearMask = ( platformClearTare ? ARM_PLATFORM : 0 ) |
                       ( shoulderClearTare ? ARM_SHOULDER : 0 ) |
                       ( elbowClearTare ? ARM_ELBOW : 0 ) |
                       ( wristClearTare ? ARM_WRIST : 0 ) |
                       ( clawClearTare ? ARM_CLAW : 0 );

  uint16_t tareMask = ( platformTare ? ARM_PLATFORM : 0 ) |
                      ( shoulderTare ? ARM_SHOULDER : 0 ) |
                      ( elbowTare ? ARM_ELBOW : 0 ) |
                      ( wristTare ? ARM_WRIST : 0 ) |
                      ( clawTare ? ARM_CLAW : 0 );
  
  ArmDataFrame frame;
  frame.values.param1 = clearMask;
  frame.values.param2 = tareMask;
  frame.values.param3 = 0;
  frame.values.param4 = 0;
  
  /*if (platformClearTare) {
    platform.imu.clearTare();
  }
  
  if (platformTare) {    
    platform.imu.tare(TARE_AXIS_ALL);
    platform.imu.saveTare();
  }*/

  twai.sendData(CAN_TARE, frame.bytes);
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