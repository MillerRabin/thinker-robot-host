#include <Wire.h>
#include "config.h"
#include "localWitmotion.h"
#include "twai.h"
#include "localINA3221.h"

class ArmPlatform {  
  private:
    DetectorsCallback callback;
  public:
    LocalWitmotion imu;
    LocalINA3221 ina;
    void begin(TwoWire &wire);
    ArmPlatform(DetectorsCallback callback) : callback(callback), 
                                              imu(Serial2,IMU_RX_GPIO, IMU_TX_GPIO, callback),
                                              ina() {};
};