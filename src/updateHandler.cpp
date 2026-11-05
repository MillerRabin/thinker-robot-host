#include <Update.h>
#include <ESPAsyncWebServer.h>
#include "updateHandler.h"
#include "powerManagement.h"

bool shouldReboot = false;

void enableUpdate() {
  // Simple Firmware Update Form
  Server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
  });

  Server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      Serial.printf("Update Start: %s\n", filename.c_str());
      // The reboot at the end cuts the engines anyway - cut them now, not mid-move.
      PowerManagement::disableEngines();
      // An upload at full speed keeps the TCP/IP task busy on core 0 while the flash is written, and
      // core 0's idle task starved: task_wdt aborted 5 s in and rebooted the host (2026-11-06) - curl
      // got "connection reset" and only a ~14 KB/s upload got through. The reboot below re-arms it.
      disableCore0WDT();

      if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(Serial);
        enableCore0WDT();
      }
    }
  });
}