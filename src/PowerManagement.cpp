#include "PowerManagement.h"

const uint PowerManagement::enginePin = ENGINE_POWER_GPIO;
const uint PowerManagement::cameraPin = CAMERA_POWER_GPIO;

bool PowerManagement::init() {  
  pinMode(cameraPin, OUTPUT);
  pinMode(enginePin, OUTPUT);
  disableCamera();
  disableEngines();
  return true;
}

bool PowerManagement::enableEngines() {
  digitalWrite(enginePin, 1);  
  return true;
}

bool PowerManagement::disableEngines() {
  digitalWrite(enginePin, 0);
  return true;
}


bool PowerManagement::enableCamera() {
  digitalWrite(cameraPin, 1);
  return true;
}

bool PowerManagement::disableCamera() {
  digitalWrite(cameraPin, 0);
  return true;
}