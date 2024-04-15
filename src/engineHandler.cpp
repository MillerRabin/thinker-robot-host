#include "engineHandler.h"

void sendSuccess(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
    
  JsonObject& imu = root.createNestedObject("imu");
  Euler se = Arm::shoulder.imu.quaternion.getEuler();
  imu["shoulder-roll"] = se.getRollAngle();
  imu["shoulder-pitch"] = se.getPitchAngle();
  imu["shoulder-yaw"] = se.getPitchAngle();
  root.printTo(*response);
  request->send(response);
}

void enableEngineHandler() {
  AsyncCallbackJsonWebHandler* engineHandler = new AsyncCallbackJsonWebHandler("/set", [](AsyncWebServerRequest *request, JsonVariant &json) mutable {    
    JsonObject& jsonObj = json.as<JsonObject>();
    Arm::set(jsonObj);    
    sendSuccess(request);
  });
  Server.addHandler(engineHandler); 
}
