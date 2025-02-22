#include "remoteWitmotion.h"

Quaternion RemoteWitmotion::getQuaternion() {
  this->quaternion.convertRawData();
  return this->quaternion;
}