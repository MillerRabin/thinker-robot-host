#pragma once

#include <string>

using namespace std;

#define RESPONSE_STATUS_SUCCESS 0
#define RESPONSE_STATUS_NO_ARM_PART_KEY_FOUND -1
#define RESPONSE_STATUS_TWAI_SEND_DATA_ERROR - 2

class StatusResponse {
  public:
    string message;
    int code; 
};