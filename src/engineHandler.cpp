#include "engineHandler.h"

void sendSuccess(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
   
  JsonObject& platform = root.createNestedObject("platform");
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

  JsonObject& shoulder = root.createNestedObject("shoulder");    
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

  JsonObject& elbow = root.createNestedObject("elbow");
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

  JsonObject& wrist = root.createNestedObject("wrist");
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
  root.printTo(*response);
  request->send(response);
}

void sendStatus(AsyncWebServerRequest *request, StatusResponse response) {
  AsyncResponseStream *resp = request->beginResponseStream("application/json");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
   
  root["message"] = response.message.c_str();
  root["code"] = response.code;
  root.printTo(*resp);
  request->send(resp);
}

void enableEngineHandler() {  
  Server.on("/set", HTTP_OPTIONS, [](AsyncWebServerRequest *request) {
    request->send(200);
  });

  AsyncCallbackJsonWebHandler* engineHandler = new AsyncCallbackJsonWebHandler("/set", [](AsyncWebServerRequest *request, JsonVariant &json) mutable {    
    JsonObject& jsonObj = json.as<JsonObject>();
    Arm::set(jsonObj);    
    sendSuccess(request);  
  });

  Server.on("/setRotate", HTTP_OPTIONS, [](AsyncWebServerRequest *request) {
    request->send(200);
  });

  AsyncCallbackJsonWebHandler* rotationHandler = new AsyncCallbackJsonWebHandler("/setRotate", [](AsyncWebServerRequest *request, JsonVariant &json) mutable {    
    JsonObject& jsonObj = json.as<JsonObject>();
    Arm::setRotate(jsonObj);    
    sendSuccess(request);
  });
  
  Server.on("/status", HTTP_OPTIONS, [](AsyncWebServerRequest *request) {  
    request->send(200);
  });
  
  AsyncCallbackJsonWebHandler* statusHandler = new AsyncCallbackJsonWebHandler("/status", [](AsyncWebServerRequest *request, JsonVariant &json) mutable {
    sendSuccess(request);
  });

  Server.on("/upgrade", HTTP_OPTIONS, [](AsyncWebServerRequest *request) {  
    request->send(200);
  });
  

  AsyncCallbackJsonWebHandler* upgradeHandler = new AsyncCallbackJsonWebHandler("/upgrade", [](AsyncWebServerRequest *request, JsonVariant &json) mutable {    
    JsonObject& jsonObj = json.as<JsonObject>();
    auto status = Arm::upgrade(jsonObj);
    sendStatus(request, status);
  });
  
  Server.addHandler(engineHandler); 
  Server.addHandler(rotationHandler); 
  Server.addHandler(statusHandler); 
  Server.addHandler(upgradeHandler); 
}
