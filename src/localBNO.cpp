#include "localBNO.h"

//Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);                                  // East North Up
//Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, 0.0f, sqrt(2.0) / 2.0f, sqrt(2.0f) / 2.0f);         // North West Up
//Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, 0.0f, 1.0f, 0.0f);                                  // West South Up
//Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, 0.0f, -sqrt(2.0f) / 2.0f, sqrt(2.0f) / 2.0f);       // South East Up
//Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, -1.0f, 0.0f, 0.0f);                                 // East South Down
//Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f, 0.0f);      // North East Down
//Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f, 0.0f, 0.0f, 0.0f);                                 // West North Down
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0) / 2.0f, sqrt(2.0) / 2.0f, 0.0f, 0);            // South West Down
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, -sqrt(2.0) / 2.0f, sqrt(2.0) / 2.0f, 0);            // Up South East
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);    // North Up East
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, 0.0f, sqrt(2.0f) / 2.0f);       // Down North East
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, 1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Down East
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, 0.0f, -sqrt(2.0f) / 2.0);       // Up North West
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f);  // North Down West
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, -sqrt(2.0f) / 2.0f, 0.0f);      // Down South West
//  Quaternion LocalBNO::rotateQuatenion = Quaternion(1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);    // South Up West
// Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, -1.0f / 2.0f);   // Up East North
// Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, sqrt(2.0f) / 2.0f, 0.0f);       // West Up North
// Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);     // Down West North
// Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0f) / 2.0f);      // East Down North
// Quaternion LocalBNO::rotateQuatenion = Quaternion(1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f, 1.0f / 2.0f);     // Up West South
// Quaternion LocalBNO::rotateQuatenion = Quaternion(-sqrt(2.0f) / 2.0f, 0.0f, -sqrt(2.0) / 2.0f, 0.0f);       // West Down South
// Quaternion LocalBNO::rotateQuatenion = Quaternion(-1.0f / 2.0f, -1.0f / 2.0f, -1.0f / 2.0f, 1.0f / 2.0f);   // Down East South
// Quaternion LocalBNO::rotateQuatenion = Quaternion(0.0f, -sqrt(2.0f) / 2.0f, 0.0f, sqrt(2.0f) / 2.0f);       // East Up South

TaskHandle_t LocalBNO::taskHandle;
SemaphoreHandle_t LocalBNO::loopMutex;
const uint32_t LocalBNO::notificationIndex = 0;

void IRAM_ATTR LocalBNO::interruptHandler() {
  if (LocalBNO::taskHandle == NULL) return;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
  vTaskNotifyGiveIndexedFromISR(LocalBNO::taskHandle, LocalBNO::notificationIndex, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool LocalBNO::initBNO() {
  (*LocalBNO::spi).end();

  pinMode(IMU_SPI_SCK, OUTPUT);
  pinMode(IMU_SPI_MOSI_GPIO, OUTPUT);
  pinMode(IMU_SPI_MISO_GPIO, INPUT);
  pinMode(IMU_SPI_CS_GPIO, OUTPUT);

  digitalWrite(IMU_SPI_CS_GPIO, HIGH);
  digitalWrite(IMU_SPI_SCK, HIGH);
  digitalWrite(IMU_SPI_MOSI_GPIO, HIGH);

  vTaskDelay(pdMS_TO_TICKS(100));  
  SPI.begin(IMU_SPI_SCK, IMU_SPI_MISO_GPIO, IMU_SPI_MOSI_GPIO, IMU_SPI_CS_GPIO);    
  if (!bno.beginSPI(IMU_SPI_CS_GPIO, IMU_WAKE_GPIO, imuIntPin, imuRstPin, IMU_SPI_SPEED, *LocalBNO::spi)) {
    Serial.printf("BNO080 not detected at SPI\n");
    return false;
  }

  interrupts();

  bno.enableRotationVector(50);
  bno.enableAccelerometer(50);
  bno.enableGyro(50);
  return true;
}

void LocalBNO::begin(SPIClass& spi, DetectorsCallback callback) {
  LocalBNO::callback = callback;
  LocalBNO::loopMutex = xSemaphoreCreateMutex();
  LocalBNO::spi = &spi;

  pinMode(imuIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(imuIntPin), interruptHandler, FALLING);
  
  xTaskCreate(
    loop,
    "LocalBNO::loop",
    4096,
    this,
    tskIDLE_PRIORITY,
    &LocalBNO::taskHandle
  );
}

void LocalBNO::loop(void* instance) {
  LocalBNO* imu = (LocalBNO*)instance ;
  uint32_t notificationValue;   
  bool needCalibration = false; 
  uint errorCount = 0;

  while(true) {
    if (!imu->initBNO()) {
      Serial.printf("Failed to initialize BNO080\n");      
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    while (errorCount < 10) {
      notificationValue = ulTaskNotifyTakeIndexed(LocalBNO::notificationIndex, pdTRUE, pdMS_TO_TICKS(100));

      auto da = imu->bno.dataAvailable();    
      if (notificationValue == 0) {
        errorCount++;      
      }
      
      if (!da) {
        continue;
      }

      errorCount = 0;
      Quaternion qr;
      qr.fromBNO(imu->bno.rawQuatI, imu->bno.rawQuatJ, imu->bno.rawQuatK, imu->bno.rawQuatReal);
      imu->quaternion = imu->rotateQuatenion * qr;
      imu->callback(CAN_PLATFORM_QUATERNION, imu->quaternion.serialize());
      imu->accelerometer.fromBNO(imu->bno.rawAccelX, imu->bno.rawAccelY, imu->bno.rawAccelZ);
      imu->callback(CAN_PLATFORM_ACCELEROMETER, imu->accelerometer.serialize());
      imu->gyroscope.fromBNO(imu->bno.rawGyroX, imu->bno.rawGyroY, imu->bno.rawGyroZ);
      imu->callback(CAN_PLATFORM_GYROSCOPE, imu->gyroscope.serialize());
      imu->accuracy.set(imu->bno.rawQuatRadianAccuracy, imu->bno.getQuatAccuracy(), imu->bno.gyroAccuracy, imu->bno.accelLinAccuracy);
      imu->callback(CAN_PLATFORM_ACCURACY, imu->accuracy.serialize());

      if (xSemaphoreTake(LocalBNO::loopMutex, pdMS_TO_TICKS(10))) {
        imu->data.rawQuaternion = imu->quaternion;
        imu->data.rawAccelerometer = imu->accelerometer;
        imu->data.rawGyroscope = imu->gyroscope;
        imu->data.rawAccuracy = imu->accuracy;
        xSemaphoreGive(LocalBNO::loopMutex);
      } else {
        Serial.println("Can't obtain loopMutex semaphore lock");
      }

      if (!needCalibration && imu->bno.quatAccuracy < 3) {
        needCalibration = true;
        imu->bno.calibrateAll();
        printf("BNO needs calibration. Starting...\n");
      }

      if (needCalibration && imu->bno.getQuatAccuracy() == 3) {
        imu->bno.endCalibration();
        vTaskDelay(pdMS_TO_TICKS(200));
        imu->bno.saveCalibration();
        printf("BNO is calibrated. Saving...\n");
        needCalibration = false;
      }
    }
    printf("Error count : %d Try to reinit BNO\n", errorCount);
    imu->hardReset();
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  Serial.println("LocalBNO loop exiting\n");
  vTaskDelete(NULL);
}

LocalBNOData LocalBNO::getLocalData() {  
  if (xSemaphoreTake(loopMutex, pdMS_TO_TICKS(IMU_WAIT_MUTEX))) {       
    xSemaphoreGive(loopMutex);
  } else {
    Serial.println("Can't obtain loop semaphore lock");
  }
  return data;
}

void LocalBNO::hardReset() {
  digitalWrite(imuRstPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));
  digitalWrite(imuRstPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));  
}

void LocalBNO::tare(uint8_t axisMask) {
  bno.tare(axisMask, 0);
}