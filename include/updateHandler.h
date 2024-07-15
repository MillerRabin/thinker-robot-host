#include <ESPAsyncWebServer.h>
#include "server.h"

#ifndef update_handler_h
#define update_handler_h

extern bool shouldReboot;
void enableUpdate();

#endif