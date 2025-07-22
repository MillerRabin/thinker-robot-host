#include "remoteBNO.h"

Quaternion RemoteBNO::getQuaternion() {
  return Quaternion(this->quaternion);
}