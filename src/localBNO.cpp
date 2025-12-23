#include "localBNO.h"

uint LocalBNO::imuIntPin = IMU_INT_GPIO;
uint LocalBNO::imuRstPin = IMU_RST_GPIO;
SPIClass* LocalBNO::spi = NULL;
BNO080 LocalBNO::bno;
DetectorsCallback LocalBNO::callback;
IMUQuaternion LocalBNO::quaternion;
LocalBNOData LocalBNO::data;

float rollOffset = 0.0f * (PI / 180.0f);
float pitchOffset = 0.0f * (PI / 180.0f);

Quaternion errorQuat = Quaternion::FromEuler(rollOffset, pitchOffset, 0.0f);
Quaternion correctionQuat = Quaternion::Conjugate(errorQuat);
Quaternion LocalBNO::rotateQuatenion = Quaternion::Multiply(correctionQuat, {-0.7071f, 0.7071f, 0.0f, 0.0f});

Accelerometer LocalBNO::accelerometer;
Gyroscope LocalBNO::gyroscope;
Accuracy LocalBNO::accuracy;
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
    NULL,
    tskIDLE_PRIORITY,
    &LocalBNO::taskHandle
  );
}

void LocalBNO::loop(void* parameters) {
  uint32_t notificationValue;   
  bool needCalibration = false; 
  uint errorCount = 0;

  while(true) {
    if (!initBNO()) {
      Serial.printf("Failed to initialize BNO080\n");      
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    while (errorCount < 10) {
      notificationValue = ulTaskNotifyTakeIndexed(LocalBNO::notificationIndex, pdTRUE, pdMS_TO_TICKS(100));

      auto da = bno.dataAvailable();    
      if (notificationValue == 0) {
        errorCount++;      
      }
      
      if (!da) {
        continue;
      }

      errorCount = 0;      
      quaternion.fromBNO(bno.rawQuatI, bno.rawQuatJ, bno.rawQuatK, bno.rawQuatReal);
      quaternion.multiplyFirst(rotateQuatenion);
      callback(CAN_PLATFORM_QUATERNION, quaternion.serialize());                      
      accelerometer.set(bno.rawAccelX, bno.rawAccelY, bno.rawAccelZ);
      callback(CAN_PLATFORM_ACCELEROMETER, accelerometer.serialize());      
      gyroscope.set(bno.rawGyroX, bno.rawGyroY, bno.rawGyroZ);
      callback(CAN_PLATFORM_GYROSCOPE, gyroscope.serialize());
      accuracy.set(bno.rawQuatRadianAccuracy, bno.getQuatAccuracy(), bno.gyroAccuracy, bno.accelLinAccuracy);
      callback(CAN_PLATFORM_ACCURACY, accuracy.serialize());

      if (xSemaphoreTake(LocalBNO::loopMutex, pdMS_TO_TICKS(10))) {
        data.rawQuaternion = Quaternion(quaternion);
        data.rawAccelerometer = Accelerometer(accelerometer);
        data.rawGyroscope = Gyroscope(gyroscope);
        data.rawAccuracy = Accuracy(accuracy);
        xSemaphoreGive(LocalBNO::loopMutex);
      } else {
        Serial.println("Can't obtain loopMutex semaphore lock");
      }

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
    printf("Error count : %d Try to reinit BNO\n", errorCount);
    hardReset();
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