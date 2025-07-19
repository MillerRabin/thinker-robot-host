#include "powerManagement.h"

const uint PowerManagement::enginePin = ENGINE_POWER_GPIO;
const uint PowerManagement::cameraPin = CAMERA_POWER_GPIO;
bool PowerManagement::enginesEnabled = false;
bool PowerManagement::cameraEnabled = false;

bool PowerManagement::begin() {  
  pinMode(cameraPin, OUTPUT);
  pinMode(enginePin, OUTPUT);
  disableCamera();
  disableEngines();
  return true;
}

bool PowerManagement::enableEngines() {
  enginesEnabled = true;
  digitalWrite(enginePin, 1);  
  return true;
}

bool PowerManagement::disableEngines() {
  enginesEnabled = false;
  digitalWrite(enginePin, 0);
  return true;
}


bool PowerManagement::enableCamera() {
  cameraEnabled = true;
  digitalWrite(cameraPin, 1);
  return true;
}

bool PowerManagement::disableCamera() {
  cameraEnabled = false;
  digitalWrite(cameraPin, 0);
  return true;
}

bool PowerManagement::getCameraStatus()
{
  return cameraEnabled;
}

bool PowerManagement::getEnginesStatus()
{
  return enginesEnabled;
}
