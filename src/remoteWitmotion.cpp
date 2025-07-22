#include "remoteWitmotion.h"

Quaternion RemoteWitmotion::getQuaternion() {  
  return Quaternion(this->quaternion);
}