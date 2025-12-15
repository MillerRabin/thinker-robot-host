#include "engineHandler.h"
#include <ArduinoJson.h>

void sendSuccess(AsyncWebServerRequest *request)
{
  DynamicJsonDocument doc(2048);

  JsonObject root = doc.to<JsonObject>();

  // --- platform ---
  JsonObject platform = root.createNestedObject("platform");
  Quaternion pquat = Arm::platform.imu.getQuaternion();
  Accuracy pa = Arm::platform.imu.accuracy;
  platform["i"] = pquat.i;
  platform["j"] = pquat.j;
  platform["k"] = pquat.k;
  platform["real"] = pquat.real;
  platform["quaternionAccuracy"] = pa.quaternionAccuracy;
  platform["quaternionRadianAccuracy"] = pa.quaternionRadAccuracy;
  platform["accelerometerAccuracy"] = pa.accelerometerAccuracy;
  platform["gyroscopeAccuracy"] = pa.gyroscopeAccuracy;

  platform["accelerometerX"] = Arm::platform.imu.accelerometer.bnoX();
  platform["accelerometerY"] = Arm::platform.imu.accelerometer.bnoY();
  platform["accelerometerZ"] = Arm::platform.imu.accelerometer.bnoZ();

  platform["gyroX"] = Arm::platform.imu.gyroscope.bnoX();
  platform["gyroY"] = Arm::platform.imu.gyroscope.bnoY();
  platform["gyroZ"] = Arm::platform.imu.gyroscope.bnoZ();

  platform["height"] = Arm::platform.ms.barometer.getHeight();
  platform["temperature"] = Arm::platform.ms.barometer.getTemperature();
  
  // --- shoulder ---
  JsonObject shoulder = root.createNestedObject("shoulder");
  Quaternion squat = Arm::shoulder.imu.getQuaternion();
  Accuracy sa = Arm::shoulder.imu.accuracy;
  shoulder["i"] = squat.i;
  shoulder["j"] = squat.j;
  shoulder["k"] = squat.k;
  shoulder["real"] = squat.real;
  shoulder["quaternionAccuracy"] = sa.quaternionAccuracy;
  shoulder["quaternionRadianAccuracy"] = sa.quaternionRadAccuracy;
  shoulder["accelerometerAccuracy"] = sa.accelerometerAccuracy;
  shoulder["gyroscopeAccuracy"] = sa.gyroscopeAccuracy;

  shoulder["accelerometerX"] = Arm::shoulder.imu.accelerometer.bnoX();
  shoulder["accelerometerY"] = Arm::shoulder.imu.accelerometer.bnoY();
  shoulder["accelerometerZ"] = Arm::shoulder.imu.accelerometer.bnoZ();
  shoulder["gyroX"] = Arm::shoulder.imu.gyroscope.bnoX();
  shoulder["gyroY"] = Arm::shoulder.imu.gyroscope.bnoY();
  shoulder["gyroZ"] = Arm::shoulder.imu.gyroscope.bnoZ();
  
  // --- elbow ---
  JsonObject elbow = root.createNestedObject("elbow");
  Quaternion equat = Arm::elbow.imu.getQuaternion();
  Accuracy ea = Arm::elbow.imu.accuracy;
  elbow["i"] = equat.i;
  elbow["j"] = equat.j;
  elbow["k"] = equat.k;
  elbow["real"] = equat.real;
  elbow["quaternionAccuracy"] = ea.quaternionAccuracy;
  elbow["quaternionRadianAccuracy"] = ea.quaternionRadAccuracy;
  elbow["accelerometerAccuracy"] = ea.accelerometerAccuracy;
  elbow["gyroscopeAccuracy"] = ea.gyroscopeAccuracy;
  
  elbow["accelerometerX"] = Arm::elbow.imu.accelerometer.bnoX();
  elbow["accelerometerY"] = Arm::elbow.imu.accelerometer.bnoY();
  elbow["accelerometerZ"] = Arm::elbow.imu.accelerometer.bnoZ();
  elbow["gyroX"] = Arm::elbow.imu.gyroscope.bnoX();
  elbow["gyroY"] = Arm::elbow.imu.gyroscope.bnoY();
  elbow["gyroZ"] = Arm::elbow.imu.gyroscope.bnoZ();

  // --- wrist ---
  JsonObject wrist = root.createNestedObject("wrist");
  Quaternion wquat = Arm::wrist.imu.getQuaternion();
  Accuracy wa = Arm::wrist.imu.accuracy;
  wrist["i"] = wquat.i;
  wrist["j"] = wquat.j;
  wrist["k"] = wquat.k;
  wrist["real"] = wquat.real;
  wrist["quaternionAccuracy"] = wa.quaternionAccuracy;
  wrist["quaternionRadianAccuracy"] = wa.quaternionRadAccuracy;
  wrist["accelerometerAccuracy"] = wa.accelerometerAccuracy;
  wrist["gyroscopeAccuracy"] = wa.gyroscopeAccuracy;
  wrist["accelerometerX"] = Arm::wrist.imu.accelerometer.bnoX();
  wrist["accelerometerY"] = Arm::wrist.imu.accelerometer.bnoY();
  wrist["accelerometerZ"] = Arm::wrist.imu.accelerometer.bnoZ();
  wrist["gyroX"] = Arm::wrist.imu.gyroscope.bnoX();
  wrist["gyroY"] = Arm::wrist.imu.gyroscope.bnoY();
  wrist["gyroZ"] = Arm::wrist.imu.gyroscope.bnoZ();

  // --- claw ---
  JsonObject claw = root.createNestedObject("claw");
  Quaternion cquat = Arm::claw.imu.getQuaternion();
  claw["i"] = cquat.i;
  claw["j"] = cquat.j;
  claw["k"] = cquat.k;
  claw["real"] = cquat.real;
  claw["distance"] = Arm::claw.range.range;
  claw["distanceMeasureType"] = Arm::claw.range.measureType;

  claw["accelerometerX"] = Arm::claw.imu.accelerometer.witMotionX();
  claw["accelerometerY"] = Arm::claw.imu.accelerometer.witMotionY();
  claw["accelerometerZ"] = Arm::claw.imu.accelerometer.witMotionZ(); 
  claw["gyroX"] = Arm::claw.imu.gyroscope.witMotionX();
  claw["gyroY"] = Arm::claw.imu.gyroscope.witMotionY();
  claw["gyroZ"] = Arm::claw.imu.gyroscope.witMotionZ();

  claw["height"] = Arm::claw.barometer.getHeight();
  claw["temperature"] = Arm::claw.barometer.getTemperature();  

  // --- status ---
  JsonObject st = root.createNestedObject("status");
  st["canSendOK"] = Arm::status.canSendOK;
  st["shoulderOK"] = Arm::status.shoulderQuaternionOK;
  st["elbowOK"] = Arm::status.elbowQuaternionOK;
  st["wristOK"] = Arm::status.wristQuaternionOK;
  st["clawOK"] = Arm::status.clawQuaternionOK;
  st["clawRangeOK"] = Arm::status.clawRangeOK;
  st["shoulderStatuses"] = Arm::status.shoulderStatuses;
  st["elbowStatuses"] = Arm::status.elbowStatuses;
  st["wristStatuses"] = Arm::status.wristStatuses;
  st["clawStatuses"] = Arm::status.clawStatuses;

  // --- powerManagement ---
  JsonObject power = root.createNestedObject("powerManagement");
  power["enginesEnabled"] = Arm::powerManagement.getEnginesStatus();
  power["cameraEnabled"] = Arm::powerManagement.getCameraStatus();

  // --- send response ---
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  serializeJson(doc, *response);
  request->send(response);
}

void sendStatus(AsyncWebServerRequest *request, const StatusResponse &response)
{
  AsyncResponseStream *resp = request->beginResponseStream("application/json");
  DynamicJsonDocument doc(256);
  doc["message"] = response.message;
  doc["code"] = response.code;
  serializeJson(doc, *resp);
  request->send(resp);
}

void enableEngineHandler()
{
  Server.on("/set", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); });

  AsyncCallbackJsonWebHandler *engineHandler = new AsyncCallbackJsonWebHandler("/set", [](AsyncWebServerRequest *request, JsonVariant json)
                                                                               {    
    if (!json.is<JsonObject>()) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON object\"}");
      return;
    }
    
    JsonObject jsonObj = json.as<JsonObject>();
    Arm::set(jsonObj);
    sendSuccess(request); });

  Server.on("/setRotate", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); });

  AsyncCallbackJsonWebHandler *rotationHandler = new AsyncCallbackJsonWebHandler("/setRotate", [](AsyncWebServerRequest *request, JsonVariant json)
                                                                                 {
    if (!json.is<JsonObject>()) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON object\"}");
      return;
    }

    JsonObject jsonObj = json.as<JsonObject>();
    Arm::setRotate(jsonObj);
    sendSuccess(request); });

  Server.on("/status", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); });

  AsyncCallbackJsonWebHandler *statusHandler = new AsyncCallbackJsonWebHandler("/status", [](AsyncWebServerRequest *request, JsonVariant &json) mutable
                                                                               { sendSuccess(request); });

  Server.on("/upgrade", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); });

  AsyncCallbackJsonWebHandler *upgradeHandler = new AsyncCallbackJsonWebHandler("/upgrade", [](AsyncWebServerRequest *request, JsonVariant json)
                                                                                {
    if (!json.is<JsonObject>()) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON object\"}");
      return;
    }

    JsonObject jsonObj = json.as<JsonObject>();
    auto status = Arm::upgrade(jsonObj);
    sendStatus(request, status); });

  Server.addHandler(engineHandler);
  Server.addHandler(rotationHandler);
  Server.addHandler(statusHandler);
  Server.addHandler(upgradeHandler);
}
