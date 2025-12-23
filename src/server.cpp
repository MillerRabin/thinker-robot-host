#include "server.h"
#include <ESPAsyncWebServer.h>

AsyncWebServer Server(80);

void onRequest(AsyncWebServerRequest *request){
  request->send(404);
}

void serverBegin() {
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  Server.onNotFound(onRequest);
  Server.begin();
}