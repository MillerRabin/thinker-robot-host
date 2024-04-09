#pragma once

#include <Arduino.h>
#include "config.h"
#include <ESP32-TWAI-CAN.hpp>

typedef void (* TWAICallback)( CanFrame frame );

class TWAI {
  private:
    const static uint rxPin;
    const static uint txPin;
    static CanFrame rxFrame;
    static TWAICallback callback;
    static void loop(void* parameters);
  public:
    static void begin(TWAICallback callback);
    static void getData();
};