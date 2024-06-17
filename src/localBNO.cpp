#include "localBNO.h"

uint LocalBNO::imuIntPin = IMU_INT_GPIO;
BNO080 LocalBNO::bno;

uint8_t LocalBNO::queueStorage[1];
StaticQueue_t LocalBNO::xStaticQueue;
QueueHandle_t LocalBNO::queueHandle;
DetectorsCallback LocalBNO::callback;
Quaternion LocalBNO::quaternion;
Accelerometer LocalBNO::accelerometer;
Gyroscope LocalBNO::gyroscope;
Accuracy LocalBNO::accuracy;

void LocalBNO::interruptHandler() {
  byte dataReady = 1;
  if (!queueHandle) return;
  xQueueSendFromISR(queueHandle, &dataReady, NULL);
}

void LocalBNO::begin(SPIClass& spi, DetectorsCallback callback) {
  LocalBNO::callback = callback;
  queueHandle = xQueueCreateStatic(1, 1, queueStorage, &xStaticQueue );  
        
  if (!bno.beginSPI(IMU_SPI_CS_GPIO, IMU_WAKE_GPIO, IMU_INT_GPIO, IMU_RST_GPIO, IMU_SPI_SPEED, spi)) {
    Serial.println(F("BNO080 not detected at SPI"));
  }
  
  attachInterrupt(digitalPinToInterrupt(imuIntPin), interruptHandler, FALLING);
  interrupts();
  
  bno.enableRotationVector(50);
  bno.enableLinearAccelerometer(50);
  bno.enableGyro(50);
  bno.clearTare();
  bno.calibrateAll();  
  bno.getReadings();
  
  xTaskCreate(
    loop,
    "LocalBNO::loop",
    4096,
    NULL,
    tskIDLE_PRIORITY,
    NULL
  );
}

void LocalBNO::printAccuracyLevel(byte accuracyNumber) {
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}

void LocalBNO::loop(void* parameters) {
  byte dataReady = false;
  while (true) {
    if (xQueueReceive(queueHandle, &dataReady, pdMS_TO_TICKS(2000)) != pdTRUE) {
      Serial.printf("No data from BNO\n");    
      continue;
    }          
    bno.getReadings();
    if (quaternion.set(bno.rawQuatI, bno.rawQuatJ, bno.rawQuatK, bno.rawQuatReal))
      callback(CAN_PLATFORM_QUATERNION, quaternion.serialize());
    if (accelerometer.set(bno.rawLinAccelX, bno.rawLinAccelY, bno.rawLinAccelZ))
      callback(CAN_PLATFORM_ACCELEROMETER, accelerometer.serialize());
    if (gyroscope.set(bno.rawGyroX, bno.rawGyroY, bno.rawGyroZ))
      callback(CAN_PLATFORM_GYROSCOPE, gyroscope.serialize());
    if (accuracy.set(bno.rawQuatRadianAccuracy, bno.quatAccuracy, bno.gyroAccuracy, bno.accelLinAccuracy))
      callback(CAN_PLATFORM_ACCURACY, gyroscope.serialize());
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