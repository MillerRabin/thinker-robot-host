#include "engineHandler.h"

void sendSuccess(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
 
  JsonObject& platform = root.createNestedObject("platform");
  Quaternion pquat = Arm::platform.imu.quaternion;
  Euler pe = pquat.getEuler();
  Accuracy pa = Arm::platform.imu.accuracy;
  platform["roll"] = pe.getRollAngle();
  platform["pitch"] = pe.getPitchAngle();
  platform["yaw"] = pe.getYawAngle();
  platform["i"] = pquat.i;
  platform["j"] = pquat.j;
  platform["k"] = pquat.k;
  platform["real"] = pquat.real;
  platform["quaternionAccuracy"] = pa.quaternionAccuracy;
  platform["quaternionRadianAccuracy"] = pa.quaternionRadAccuracy;
  platform["accelerometerAccuracy"] = pa.accelerometerAccuracy;
  platform["gyroscopeAccuracy"] = pa.gyroscopeAccuracy;

  JsonObject& shoulder = root.createNestedObject("shoulder");    
  Quaternion squat = Arm::shoulder.imu.quaternion;  
  Euler se = squat.getEuler();  
  Accuracy sa = Arm::shoulder.imu.accuracy;
  shoulder["roll"] = se.getRollAngle();
  shoulder["pitch"] = se.getPitchAngle();
  shoulder["yaw"] = se.getYawAngle();
  shoulder["i"] = squat.i;
  shoulder["j"] = squat.j;
  shoulder["k"] = squat.k;
  shoulder["real"] = squat.real;
  shoulder["quaternionAccuracy"] = sa.quaternionAccuracy;
  shoulder["quaternionRadianAccuracy"] = sa.quaternionRadAccuracy;
  shoulder["accelerometerAccuracy"] = sa.accelerometerAccuracy;
  shoulder["gyroscopeAccuracy"] = sa.gyroscopeAccuracy;
  root.printTo(*response);
  request->send(response);
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
  
  
  Server.addHandler(engineHandler); 
  Server.addHandler(rotationHandler); 
  Server.addHandler(statusHandler); 
}
