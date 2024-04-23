#include "armPlatform.h"

LocalBNO ArmPlatform::bno;
PMS5611 ArmPlatform::ms;

void ArmPlatform::begin(TwoWire& wire, SPIClass& spi) {
  ms.begin(wire);
  bno.begin(spi);
}