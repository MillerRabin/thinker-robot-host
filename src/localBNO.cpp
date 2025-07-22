#include "localBNO.h"

uint LocalBNO::imuIntPin = IMU_INT_GPIO;
BNO080 LocalBNO::bno;
DetectorsCallback LocalBNO::callback;
IMUQuaternion LocalBNO::quaternion;
Quaternion LocalBNO::lastQuaternion;

float rollOffset = 0.9f * (PI / 180.0f);
float pitchOffset = -1.680f * (PI / 180.0f);

Quaternion errorQuat = Quaternion::FromEuler(rollOffset, pitchOffset, 0.0f);
Quaternion correctionQuat = Quaternion::Conjugate(errorQuat);
Quaternion LocalBNO::rotateQuatenion = Quaternion::Multiply(correctionQuat, {-0.7071f, 0.7071f, 0.0f, 0.0f});

// Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.7071f, 0.0f, 0.0f};
// Quaternion LocalBNO::rotateQuatenion = {-0.0098599f, 0.0146595f, -0.0001445f, 0.9998439f};

/// East-North-Up (ENU)
//Quaternion LocalBNO::rotateQuatenion = {0.0f, 0.0f, 0.0f, 1.0f};

/// North-West-Up
//Quaternion LocalBNO::rotateQuatenion = {0.0f, 0.0f, 0.7071f, 0.7071f};

/// West-South-Up
//Quaternion LocalBNO::rotateQuatenion = {0.0f, 0.0f, 1.0f, 0.0f};

/// South-East-Up
//Quaternion LocalBNO::rotateQuatenion = {0.0f, 0.0f, -0.7071f, 0.7071f};

/// East-South-Down
//Quaternion LocalBNO::rotateQuatenion = {0.0f, -1.0f, 0.0f, 0.0f};

/// North-East-Down
//Quaternion LocalBNO::rotateQuatenion = {-0.7071f, -0.7071f, 0.0f, 0.0f};

/// West-North-Down
//Quaternion LocalBNO::rotateQuatenion = {-1.0f, 0.0f, 0.0f, 0.0f};

/// South-West-Down
//Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.7071f, 0.0f, 0.0f};

/// Up-South-East
// Quaternion LocalBNO::rotateQuatenion = {0.0f, -0.7071f, 0.7071f, 0.0f};

/// North-Up-East
// Quaternion LocalBNO::rotateQuatenion = {-0.5f, -0.5f, 0.5f, 0.5f};

/// Down-North-East
// Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.0f, 0.0f, 0.7071f};

/// South-Down-East
// Quaternion LocalBNO::rotateQuatenion = {0.5f, -0.5f, 0.5f, -0.5f};

/// Up-North-West
// Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.0f, 0.0f, -0.7071f};

/// North-Down-West
// Quaternion LocalBNO::rotateQuatenion = {-0.5f, -0.5f, -0.5f, -0.5f};

/// Down-South-West
// Quaternion LocalBNO::rotateQuatenion = {0.0f, -0.7071f, -0.7071f, 0.0f};

/// South-Up-West
// Quaternion LocalBNO::rotateQuatenion = {0.5f, -0.5f, 0.5f, 0.5f};

/// Up-East-North
// Quaternion LocalBNO::rotateQuatenion = {-0.5f, -0.5f, -0.5f, 0.5f};

/// West-Up-North
// Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.0f, 0.7071f, 0.0f};

/// Down-West-North
// Quaternion LocalBNO::rotateQuatenion = {0.5f, -0.5f, 0.5f, 0.5f};

/// East-Down-North
// Quaternion LocalBNO::rotateQuatenion = {0.0f, -0.7071f, 0.0f, -0.7071f};

/// Up-West-South
// Quaternion LocalBNO::rotateQuatenion = {0.5f, -0.5f, 0.5f, -0.5f};

/// West-Down-South
// Quaternion LocalBNO::rotateQuatenion = {-0.7071f, 0.0f, -0.7071f, 0.0f};

/// Down-East-South
// Quaternion LocalBNO::rotateQuatenion = {-0.5f, -0.5f, -0.5f, -0.5f};

/// East-Up-South
// Quaternion LocalBNO::rotateQuatenion = {0.0f, -0.7071f, 0.0f, 0.7071f};

Accelerometer LocalBNO::accelerometer;
Gyroscope LocalBNO::gyroscope;
Accuracy LocalBNO::accuracy;
TaskHandle_t LocalBNO::taskHandle;
SemaphoreHandle_t LocalBNO::loopMutex;
const uint32_t LocalBNO::notificationIndex = 0;

void LocalBNO::interruptHandler() {
  if (LocalBNO::taskHandle == NULL) return;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
  vTaskNotifyGiveIndexedFromISR(LocalBNO::taskHandle, LocalBNO::notificationIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LocalBNO::initBNO() {
  bno.enableRotationVector(50);
  bno.enableLinearAccelerometer(50);
  bno.enableGyro(50);
}

void LocalBNO::begin(SPIClass& spi, DetectorsCallback callback) {
  LocalBNO::callback = callback;
  LocalBNO::loopMutex = xSemaphoreCreateMutex();
  if (!bno.beginSPI(IMU_SPI_CS_GPIO, IMU_WAKE_GPIO, IMU_INT_GPIO, IMU_RST_GPIO, IMU_SPI_SPEED, spi)) {
    Serial.println(F("BNO080 not detected at SPI"));
    return;
  }

  /*uint32_t quaternionFRS[4] = {
    StructureBase::floatToQ32(rotateQuatenion.real, 30),
    StructureBase::floatToQ32(rotateQuatenion.i, 30),
    StructureBase::floatToQ32(rotateQuatenion.j, 30),
    StructureBase::floatToQ32(rotateQuatenion.k, 30)
  };

  printf("Writing real: 0x%x, i: 0x%x, j:0x%x, k:0x%x\n", quaternionFRS[0], quaternionFRS[1], quaternionFRS[2], quaternionFRS[3]);
  if (bno.writeFRSRecord(0x2D3E, quaternionFRS, 4)) {
    printf("FRS Orientation is written\n");
  } else {
    printf("Can't save FRS Orientation\n");
  }*/

  attachInterrupt(digitalPinToInterrupt(imuIntPin), interruptHandler, FALLING);
  interrupts();
  
  initBNO();

  bno.getReadings();
  
  xTaskCreate(
    loop,
    "LocalBNO::loop",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    &LocalBNO::taskHandle
  );
}

void LocalBNO::loop(void* parameters) {
  uint32_t notificationValue;   
  bool needCalibration = false; 
  while (true) {
    if (bno.hasReset()) {
      printf("IMU was reset. Re-enabling sensor...\n");
      initBNO();
    }
    bno.getReadings();    
    notificationValue = ulTaskNotifyTakeIndexed(LocalBNO::notificationIndex, pdTRUE, pdMS_TO_TICKS(50));      
    if(notificationValue == 0) {
      printf("LocalBNO notificationResult error %d\n", notificationValue);
      continue;                
    }
    if (xSemaphoreTake(LocalBNO::loopMutex, pdMS_TO_TICKS(IMU_WAIT_MUTEX))) {
      if (quaternion.setBNO(bno.rawQuatI, bno.rawQuatJ, bno.rawQuatK, bno.rawQuatReal)) {
        quaternion.multiplyFirst(rotateQuatenion);
        callback(CAN_PLATFORM_QUATERNION, quaternion.serialize());
      }
      if (accelerometer.set(bno.rawLinAccelX, bno.rawLinAccelY, bno.rawLinAccelZ))
        callback(CAN_PLATFORM_ACCELEROMETER, accelerometer.serialize());
      if (gyroscope.set(bno.rawGyroX, bno.rawGyroY, bno.rawGyroZ))
        callback(CAN_PLATFORM_GYROSCOPE, gyroscope.serialize());
      if (accuracy.set(bno.rawQuatRadianAccuracy, bno.getQuatAccuracy(), bno.gyroAccuracy, bno.accelLinAccuracy))
        callback(CAN_PLATFORM_ACCURACY, accuracy.serialize());
      xSemaphoreGive(LocalBNO::loopMutex);

      if (!needCalibration && bno.quatAccuracy < 3) {
        needCalibration = true;        
        bno.calibrateAll();
        printf("BNO needs calibration. Starting...\n");
      }

      if (needCalibration && bno.getQuatAccuracy() == 3) {
        bno.endCalibration();
        vTaskDelay(pdMS_TO_TICKS(200));
        bno.saveCalibration();
        printf("BNO is calibrated. Saving...\n");
        needCalibration = false;
      }
    }
  }
}

void LocalBNO::printData() {
  float roll = (bno.getRoll()) * 180.0 / PI;   // Convert roll to degrees
  float pitch = (bno.getPitch()) * 180.0 / PI; // Convert pitch to degrees
  float yaw = (bno.getYaw()) * 180.0 / PI;     // Convert yaw / heading to degrees*/
  Serial.print("Euler: ");
  Serial.print(roll, 1);
  Serial.print(F(","));
  Serial.print(pitch, 1);
  Serial.print(F(","));
  Serial.print(yaw, 1);
  Serial.println("");
}

Quaternion LocalBNO::getQuaternion() {  
  if (xSemaphoreTake(LocalBNO::loopMutex, pdMS_TO_TICKS(IMU_WAIT_MUTEX))) {     
    lastQuaternion = Quaternion(quaternion);
    xSemaphoreGive(LocalBNO::loopMutex);
  } else {
    Serial.println("Can't obtain loop semaphore lock");
  }
  return lastQuaternion;
}