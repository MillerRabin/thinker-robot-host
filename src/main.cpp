#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

//Set your wi-fi credentials in credentials.h
#include "credentials.h"
#include "arm.h"
#include "updateHandler.h"
#include "engineHandler.h"
#include "server.h"
#include "config.h"


const char* appVersion = APP_VERSION;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;

Arm arm;

bool enableWIFI() {  
  WiFi.mode(WIFI_STA);  
  WiFi.begin(ssid, password);   
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {      
    Serial.printf("WiFi Failed!\n");
    return false;
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void enableVersion() {
  Server.on("/version", HTTP_POST, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["version"] = appVersion;
    root["chipModel"] = ESP.getChipModel();
    root["free-heap"] = ESP.getFreeHeap();
    root["cores"] = ESP.getChipCores();
    root["frequencyMHz"] = ESP.getCpuFreqMHz();
    root["flashChipSize"] = ESP.getFlashChipSize();
    root.printTo(*response);
    request->send(response);
  });
}

void setup(){
  Serial.begin(115200);  
  if (!enableWIFI()) {
    Serial.println("Can't enable wifi");
  }
           
  enableVersion();
  enableUpdate();  
  serverBegin();
  enableEngineHandler();
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
  arm.begin(Wire);
}

void loop() {
   if(shouldReboot){
    Serial.println("Rebooting...");    
    ESP.restart();
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
}
