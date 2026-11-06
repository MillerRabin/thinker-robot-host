#include <Arduino.h>
#include <esp_system.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>

//Set your wi-fi credentials in credentials.h
#include "credentials.h"
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

// Why the host last restarted - an unexplained engines-off on 2026-11-06 (the modules saw a CAN
// glitch, then no platform status) looked like a reboot, and nothing could tell without opening the
// serial port, which resets it.
static const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

void enableVersion() {
  Server.on("/version", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonDocument doc(384);
    doc["version"] = appVersion;
    doc["build"] = BUILD_REV;
    doc["chipModel"] = ESP.getChipModel();
    doc["free-heap"] = ESP.getFreeHeap();
    doc["cores"] = ESP.getChipCores();
    doc["frequencyMHz"] = ESP.getCpuFreqMHz();
    doc["flashChipSize"] = ESP.getFlashChipSize();
    doc["uptimeSec"] = millis() / 1000;
    doc["resetReason"] = resetReasonName(esp_reset_reason());

    serializeJson(doc, *response);
    request->send(response); 
  });
}

void setup(){
  Serial.begin(115200);  
  Serial2.begin(9600, IMU_RX_GPIO, IMU_TX_GPIO);
  if (!enableWIFI()) {
    Serial.println("Can't enable wifi");
  }

  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
  arm.begin(Wire);

  enableVersion();
  enableUpdate();  
  serverBegin();
  enableEngineHandler();
}

void loop() {
   if(shouldReboot){
    Serial.println("Rebooting...");    
    ESP.restart();
  }
  cleanupClients();
  vTaskDelay(pdMS_TO_TICKS(2000));
}
