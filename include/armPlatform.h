#include <Wire.h>

#include "localBNO.h"
#include "twai.h"
#include "pms5611.h"
#include "structures.h"


class ArmPlatform {
  private:
    static LocalBNO bno;    
    static PMS5611 ms;    
  public:
    static void begin(TwoWire& wire, SPIClass& spi, DetectorsCallback callback); 
};