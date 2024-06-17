#include "armPlatform.h"

LocalBNO ArmPlatform::imu;
PMS5611 ArmPlatform::ms;

void ArmPlatform::begin(TwoWire& wire, SPIClass& spi, DetectorsCallback callback) {
  ms.begin(wire, callback);
  imu.begin(spi, callback);
}