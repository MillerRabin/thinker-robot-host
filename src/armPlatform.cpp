#include "armPlatform.h"
#include "i2cScan.h"

void ArmPlatform::begin(TwoWire& wire) {  
  imu.begin();
  if (!ina.begin(wire, callback)) {
    printf("INA3221 is not initialized.\n");
  }
}