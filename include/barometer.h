#pragma once

#include <Arduino.h>

class Barometer {
  private:
    uint8_t temperatureRaw;
    uint16_t pressureRaw;
    uint16_t heightRaw;
  public:
    uint64_t serialize();
    void deserialize(uint8_t data[8]);
    void set(uint16_t height, uint16_t pressure, uint8_t temperature);
    float getPressure() { return pressureRaw / 100.0f; };
    float getTemperature() { return temperatureRaw / 100.0f; };
    float getHeight() { return heightRaw / 100.0f; };
};
