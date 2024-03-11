#include "powerManagement.h"

#define ENGINE_PIN gpio_num_t(CONFIG_ENGINE_PIN)
#define CAMERA_PIN gpio_num_t(CONFIG_CAMERA_PIN)

bool PowerManagement::init() {
  gpio_reset_pin(ENGINE_PIN);
  gpio_reset_pin(CAMERA_PIN);
  gpio_set_direction(ENGINE_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(CAMERA_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(ENGINE_PIN, 0);
  gpio_set_level(CAMERA_PIN, 0);
  return true;
}

bool PowerManagement::enableEngines() {
  gpio_set_level(ENGINE_PIN, 1);
  return true;
}

bool PowerManagement::disableEngines() {
  gpio_set_level(ENGINE_PIN, 0);
  return true;
}


bool PowerManagement::enableCamera() {
  gpio_set_level(CAMERA_PIN, 1);
  return true;
}

bool PowerManagement::disableCamera() {
  gpio_set_level(CAMERA_PIN, 0);
  return true;
}