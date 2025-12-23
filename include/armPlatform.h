#include <Wire.h>

#include "localBNO.h"
#include "twai.h"
#include "pms5611.h"

class ArmPlatform {  
  public:
    static LocalBNO imu;
    static PMS5611 ms;    
    static void begin(TwoWire& wire, SPIClass& spi, DetectorsCallback callback); 
};