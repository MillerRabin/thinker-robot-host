#pragma once

#include "remoteBNO.h"

class ArmShoulder {
  private:
    static RemoteBNO bno;
  public:
    static void begin();
};