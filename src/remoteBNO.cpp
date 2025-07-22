#include "remoteBNO.h"

Quaternion RemoteBNO::getQuaternion(){
  this->quaternion.convertRawData();
  return Quaternion(this->quaternion);
}