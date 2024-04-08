#pragma once

#include <Arduino.h>
#include "config.h"
#include <ESP32-TWAI-CAN.hpp>

class TWAI {
  private:
    const static uint rxPin;
    const static uint txPin;
    static CanFrame rxFrame;
  public:
    static void begin();
    static void getData();
};