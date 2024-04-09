#include <Wire.h>

#include "localBNO.h"
#include "twai.h"
#include "pms5611.h"


class ArmPlatform {
  private:
    static LocalBNO bno;    
    static PMS5611 ms;
  public:
    static void begin(TwoWire& wire); 
};