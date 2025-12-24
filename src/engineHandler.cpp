#include "engineHandler.h"

TaskHandle_t WebSocketTaskHandle = NULL;
AsyncWebSocket WebSocket("/ws");

void sendStatus(AsyncWebServerRequest *request, const StatusResponse &response) {
  AsyncResponseStream *resp = request->beginResponseStream("application/json");
  DynamicJsonDocument doc(256);
  doc["message"] = response.message;
  doc["code"] = response.code;
  serializeJson(doc, *resp);
  request->send(resp);
}

void socketLoop(void *parameters)
{  
  while (true) {
    ClientSocketResponse response;

    response.status = (uint64_t)Arm::status.canSendOK |
                      (uint64_t)Arm::status.shoulderQuaternionOK << 1 |
                      (uint64_t)Arm::status.elbowQuaternionOK << 2 |
                      (uint64_t)Arm::status.wristQuaternionOK << 3 |
                      (uint64_t)Arm::status.clawQuaternionOK << 4 |
                      (uint64_t)Arm::status.clawRangeOK << 5 |
                      (uint64_t)Arm::powerManagement.getEnginesStatus() << 6 |
                      (uint64_t)Arm::powerManagement.getCameraStatus() << 7;

    auto pdata = Arm::platform.imu.getLocalData();
    auto pquat = pdata.getQuaternion();
    auto pacc = pdata.getAccuracy();
    auto pa = pdata.getAccelerometer();
    auto pgyro = pdata.getGyroscope();
    auto pmsData = Arm::platform.ms.getLocalData();
    auto pbar = pmsData.getBarometer();    
    response.platform.quaternion = pquat.serialize();
    response.platform.accelerometer = pa.serialize();
    response.platform.gyroscope = pgyro.serialize();
    response.platform.accuracy = pacc.serialize();
    response.platform.barometer = pbar.serialize();

    auto squat = Arm::shoulder.imu.getQuaternion();
    auto sa = Arm::shoulder.imu.accuracy;
    auto sacc = Arm::shoulder.imu.accelerometer;
    auto sgyro = Arm::shoulder.imu.gyroscope;
    response.shoulder.quaternion = squat.serialize();
    response.shoulder.accelerometer = sacc.serialize();
    response.shoulder.gyroscope = sgyro.serialize();
    response.shoulder.accuracy = sa.serialize();

    auto equat = Arm::elbow.imu.getQuaternion();
    auto ea = Arm::elbow.imu.accuracy;
    auto eacc = Arm::elbow.imu.accelerometer;
    auto egyro = Arm::elbow.imu.gyroscope;
    response.elbow.quaternion = equat.serialize();
    response.elbow.accelerometer = eacc.serialize();
    response.elbow.gyroscope = egyro.serialize();
    response.elbow.accuracy = ea.serialize();

    auto wquat = Arm::wrist.imu.getQuaternion();
    auto wa = Arm::wrist.imu.accuracy;
    auto wacc = Arm::wrist.imu.accelerometer;
    auto wgyro = Arm::wrist.imu.gyroscope;
    response.wrist.quaternion = wquat.serialize();
    response.wrist.accelerometer = wacc.serialize();
    response.wrist.gyroscope = wgyro.serialize();
    response.wrist.accuracy = wa.serialize();

    auto cquat = Arm::claw.imu.getQuaternion();    
    auto cacc = Arm::claw.imu.accelerometer;
    auto cgyro = Arm::claw.imu.gyroscope;
    response.claw.quaternion = cquat.serialize();
    response.claw.accelerometer = cacc.serialize();
    response.claw.gyroscope = cgyro.serialize();        
    response.claw.range = Arm::claw.range.serialize();
    response.claw.barometer = Arm::claw.barometer.serialize();

    WebSocket.binaryAll(reinterpret_cast<uint8_t *>(&response), sizeof(response));
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}

void enableEngineHandler() {
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
    auto status = StatusResponse({"Set Success", RESPONSE_STATUS_SUCCESS});    
    sendStatus(request, status); 
  });

  Server.on("/setRotate", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); 
  });

  AsyncCallbackJsonWebHandler *rotationHandler = new AsyncCallbackJsonWebHandler("/setRotate", [](AsyncWebServerRequest *request, JsonVariant json)
                                                                                 {
    if (!json.is<JsonObject>()) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON object\"}");
      return;
    }

    JsonObject jsonObj = json.as<JsonObject>();
    Arm::setRotate(jsonObj);
    auto status = StatusResponse({"Set Success", RESPONSE_STATUS_SUCCESS});
    sendStatus(request, status); 
  });

  Server.on("/upgrade", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            { request->send(200); 
  });

  AsyncCallbackJsonWebHandler *upgradeHandler = new AsyncCallbackJsonWebHandler("/upgrade", [](AsyncWebServerRequest *request, JsonVariant json){
    if (!json.is<JsonObject>()) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON object\"}");
      return;
    }

    JsonObject jsonObj = json.as<JsonObject>();
    auto status = Arm::upgrade(jsonObj);
    sendStatus(request, status); 
  });

  Server.addHandler(engineHandler);
  Server.addHandler(rotationHandler);
  Server.addHandler(upgradeHandler);  
  Server.addHandler(&WebSocket);

  auto sockerTaskResult = xTaskCreate(socketLoop, "EngineHandler::socketLoop", 4096, NULL, tskIDLE_PRIORITY, &WebSocketTaskHandle);
  if (sockerTaskResult != pdPASS) {
    Serial.println("WebSocket task creation failed!");
  }
}

void cleanupClients() {
  WebSocket.cleanupClients();
}