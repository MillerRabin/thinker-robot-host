#pragma once
#include <Arduino.h>
#include "config.h"

class PowerManagement {
  private:
    static const uint enginePin;
    static const uint cameraPin;
    static const uint detectorsDisablePin;
    static bool enginesEnabled;
    static bool cameraEnabled;
    static bool detectorsEnabled;

  public:
    static bool begin();
    static bool enableEngines();
    static bool disableEngines();
    static bool enableCamera();    
    static bool disableCamera();    
    static bool enableDetectors();
    static bool disableDetectors();
    static bool getEnginesStatus();
    static bool getCameraStatus();
    static bool getDetectorsStatus();
};