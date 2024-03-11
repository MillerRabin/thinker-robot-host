#pragma once
#include "sdkconfig.h"
#include "driver/gpio.h"

class PowerManagement {  
  public:
    static bool init();
    static bool enableEngines();
    static bool disableEngines();
    static bool enableCamera();
    static bool disableCamera();
};