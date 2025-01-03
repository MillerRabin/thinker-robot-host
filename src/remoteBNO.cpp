#include "remoteBNO.h"

Quaternion RemoteBNO::getQuaternion(){
  this->quaternion.convertRawData();
  return this->quaternion;
}