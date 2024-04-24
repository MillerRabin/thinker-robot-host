#include "armPlatform.h"

LocalBNO ArmPlatform::bno;
PMS5611 ArmPlatform::ms;

void ArmPlatform::begin(TwoWire& wire, SPIClass& spi, DetectorsCallback callback) {
  ms.begin(wire, callback);
  bno.begin(spi, callback);
}