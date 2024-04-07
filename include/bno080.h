#ifndef bno_h
#define bno_h

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

void bno_setup(TwoWire *wire, uint8_t imuIntPin);
void bno_getData();
void bno_calibrate();
void bno_calibrate_magnetometer_loop();

#endif