#include "localBNO.h"

uint LocalBNO::imuIntPin = IMU_INT_GPIO;
BNO080 LocalBNO::bno;

uint8_t LocalBNO::queueStorage[1];
StaticQueue_t LocalBNO::xStaticQueue;
QueueHandle_t LocalBNO::queueHandle;

void LocalBNO::interruptHandler() {
  byte dataReady = 1;
  if (!queueHandle) return;
  xQueueSendFromISR(queueHandle, &dataReady, NULL);
}

void LocalBNO::begin() {
  queueHandle = xQueueCreateStatic(1, 1, queueStorage, &xStaticQueue );  
  Wire.flush();
      
  if (!bno.begin(BNO080_DEFAULT_ADDRESS, Wire, imuIntPin)) {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));    
  }
  
  attachInterrupt(digitalPinToInterrupt(imuIntPin), interruptHandler, FALLING);
  interrupts();
  
  bno.enableRotationVector(50);
  bno.calibrateAll();  
  bno.getReadings();
  
  xTaskCreate(
      loop,
      "LocalBNOLoop",
      4096,
      NULL,
      tskIDLE_PRIORITY,
      NULL);
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
    }          
    bno.getReadings();    
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