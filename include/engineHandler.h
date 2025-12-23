#pragma once

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include "server.h"
#include "arm.h"
#include "qBase.h"
#include "quaternion.h"
#include "accelerometer.h"
#include "accuracy.h"

struct __attribute__((packed)) PlatformSocketResponse {  
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;
  uint64_t barometer;
};

struct __attribute__((packed)) ShoulderSocketResponse {  
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;  
};

struct __attribute__((packed)) ElbowSocketResponse {
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;
};

struct __attribute__((packed)) WristSocketResponse {
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;
};

struct __attribute__((packed)) ClawSocketResponse {
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t range;
  uint64_t barometer;
};

struct __attribute__((packed)) ClientSocketResponse {
  uint64_t status;
  PlatformSocketResponse platform;
  ShoulderSocketResponse shoulder;
  ElbowSocketResponse elbow;
  WristSocketResponse wrist;
  ClawSocketResponse claw;
};

void enableEngineHandler();
void cleanupClients();