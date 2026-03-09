#include "powerManagement.h"

const uint PowerManagement::enginePin = ENGINE_POWER_GPIO;
const uint PowerManagement::cameraPin = CAMERA_POWER_GPIO;
const uint PowerManagement::detectorsDisablePin = DETECTORS_DISABLE_GPIO;
const uint PowerManagement::peripheralCPUPowerPin = PERIPHERAL_CPU_ENABLE_GPIO;


bool PowerManagement::enginesEnabled = false;
bool PowerManagement::cameraEnabled = false;
bool PowerManagement::detectorsEnabled = true;
bool PowerManagement::cpuPowerEnabled = false;

bool PowerManagement::begin() {
  pinMode(cameraPin, OUTPUT);
  pinMode(enginePin, OUTPUT);
  pinMode(detectorsDisablePin, OUTPUT);
  pinMode(peripheralCPUPowerPin, OUTPUT);
  disableCamera();
  disableEngines();
  enableDetectors();
  disableCPUPower();
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

bool PowerManagement::getCameraStatus() { return cameraEnabled; }
bool PowerManagement::getEnginesStatus() { return enginesEnabled; }
bool PowerManagement::getDetectorsStatus() { return detectorsEnabled; }
bool PowerManagement::getCPUPowerStatus() { return cpuPowerEnabled; }

bool PowerManagement::enableDetectors() {
  digitalWrite(detectorsDisablePin, 1);
  detectorsEnabled = true;
  return true;
}

bool PowerManagement::disableDetectors() {
  digitalWrite(detectorsDisablePin, 0);
  detectorsEnabled = false;
  return true;
}

bool PowerManagement::enableCPUPower() {
  digitalWrite(peripheralCPUPowerPin, 1);
  cpuPowerEnabled = true;
  return true;
}

bool PowerManagement::disableCPUPower() {
  digitalWrite(peripheralCPUPowerPin, 0);
  cpuPowerEnabled = false;
  return true;
}
