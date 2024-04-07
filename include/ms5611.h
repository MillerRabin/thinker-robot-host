#ifndef ms5611_h
#define ms5611_h

#include <Wire.h>
#include <MS5611.h>

void ms5611_setup(TwoWire *wire);
void ms5611_getData();

#endif