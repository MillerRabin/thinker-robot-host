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
  uint8_t tag = SOCKET_PLATFORM_TAG;
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;  
  uint64_t barometer;
  uint64_t cpuPower;
  uint64_t enginesPower;
};

struct __attribute__((packed)) ShoulderSocketResponse {  
  uint8_t tag = SOCKET_SHOULDER_TAG;
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;  
};

struct __attribute__((packed)) ElbowSocketResponse {
  uint8_t tag = SOCKET_ELBOW_TAG;
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;
};

struct __attribute__((packed)) WristSocketResponse {
  uint8_t tag = SOCKET_WRIST_TAG;
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t accuracy;
};

struct __attribute__((packed)) ClawSocketResponse {
  uint8_t tag = SOCKET_CLAW_TAG;
  uint64_t quaternion;
  uint64_t accelerometer;
  uint64_t gyroscope;
  uint64_t range;
  uint64_t barometer;
};

struct __attribute__((packed)) ClientSocketResponse {
  uint8_t tag = SOCKET_RESPONSE_TAG;
  uint64_t status;
  PlatformSocketResponse platform;
  ShoulderSocketResponse shoulder;
  ElbowSocketResponse elbow;
  WristSocketResponse wrist;
  ClawSocketResponse claw;
};

void enableEngineHandler();
void cleanupClients();