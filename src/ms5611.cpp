
#include "ms5611.h"

MS5611 MS5611(0x77);
uint32_t start, stop;
void ms5611_setup(TwoWire *wire)
{
  if (MS5611.begin() == true) {
    Serial.println("MS5611 found.");
  } else {
    Serial.println("MS5611 is not found.");
  }
  MS5611.reset(1);  
}

void test()
{
  start = micros();
  int result = MS5611.read();
  stop = micros();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
  else
  {
    Serial.print("T:\t");
    Serial.print(MS5611.getTemperature(), 2);
    Serial.print("\tP:\t");
    Serial.print(MS5611.getPressure(), 2);
    Serial.print("\tt:\t");
    Serial.print(stop - start);
    Serial.println();
  }
}

/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
*/
void ms5611_getData()
{    
  MS5611.setOversampling(OSR_ULTRA_HIGH);
  test();  
}

