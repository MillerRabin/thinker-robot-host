#include "remoteWitmotion.h"

Quaternion RemoteWitmotion::getQuaternion() {
  this->quaternion.convertRawDataByDivision(32768.0);
  return this->quaternion;
}