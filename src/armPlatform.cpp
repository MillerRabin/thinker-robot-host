#include "armPlatform.h"

LocalBNO ArmPlatform::bno;
PMS5611 ArmPlatform::ms;

void ArmPlatform::begin(TwoWire& wire) {
  ms.begin();
  bno.begin(wire);
}