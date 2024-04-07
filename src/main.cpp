#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

//Set your wi-fi credentials in credentials.h
#include "credentials.h"
#include "updateHandler.h"
#include "initServer.h"
#include "armParams.h"

const char* appVersion = APP_VERSION;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* http_username = HTTP_USERNAME;
const char* http_password = HTTP_PASSWORD;

bool enableWIFI() {
  Serial.printf("Try to enable wifi\n");
  WiFi.mode(WIFI_STA);
  Serial.printf("WiFI is set\n");
  /*WiFi.begin(ssid, password);   
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {      
    Serial.printf("WiFi Failed!\n");
    return false;
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());*/
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
  Serial.println("Setup");
  if (!enableWIFI())
      Serial.println("Can't enable wifi");
       
  //enableVersion();
  //enableUpdate();  
  //serverBegin();
}

void loop() {
   if(shouldReboot){
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }
  Serial.println("Loop");
  delay(1000);
}