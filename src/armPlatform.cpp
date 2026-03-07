#include "armPlatform.h"
#include "i2cScan.h"

void ArmPlatform::begin(TwoWire& wire) {  
  imu.begin();
  ina.begin(wire, callback);  
}