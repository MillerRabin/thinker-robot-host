#pragma once
#include <Arduino.h>
#include "config.h"

class PowerManagement {
  private:
    static const uint enginePin;
    static const uint cameraPin;
    static bool enginesEnabled;
    static bool cameraEnabled;

  public:
    static bool begin();
    static bool enableEngines();
    static bool disableEngines();
    static bool enableCamera();
    static bool disableCamera();
    static bool getEnginesStatus();
    static bool getCameraStatus();
};