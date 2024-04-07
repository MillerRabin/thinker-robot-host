#include <bno080.h>

BNO080 myIMU;
bool accReady = false;
bool quatReady = false;

void bno_printStats()
{
  int rotationVector_Q1 = myIMU.getQ1(FRS_RECORDID_ROTATION_VECTOR);
  int accelerometer_Q1 = myIMU.getQ1(FRS_RECORDID_ACCELEROMETER);

  Serial.println();
  Serial.println("For rotation vector");
  Serial.print("Range: ");
  Serial.println(myIMU.getRange(FRS_RECORDID_ROTATION_VECTOR), 4);
  Serial.print("Resolution: ");
  Serial.println(myIMU.getResolution(FRS_RECORDID_ROTATION_VECTOR), 10);
  Serial.print("Q1: ");
  Serial.println(myIMU.getQ1(FRS_RECORDID_ROTATION_VECTOR));
  Serial.print("Q2: ");
  Serial.println(myIMU.getQ2(FRS_RECORDID_ROTATION_VECTOR));
  Serial.print("Q3: ");
  Serial.println(myIMU.getQ3(FRS_RECORDID_ROTATION_VECTOR));

  Serial.println();
  Serial.println("For accelerometer");
  Serial.print("Range: ");
  Serial.println(myIMU.getRange(FRS_RECORDID_ACCELEROMETER), 4);
  Serial.print("Resolution: ");
  Serial.println(myIMU.getResolution(FRS_RECORDID_ACCELEROMETER), 4);
  Serial.print("Q1: ");
  Serial.println(myIMU.getQ1(FRS_RECORDID_ACCELEROMETER));
  Serial.print("Q2: ");
  Serial.println(myIMU.getQ2(FRS_RECORDID_ACCELEROMETER));
  Serial.print("Q3: ");
  Serial.println(myIMU.getQ3(FRS_RECORDID_ACCELEROMETER));

  //Example of reading meta data manually
  //See page 30 of the reference manual
  Serial.println();
  uint16_t accelerometer_power = myIMU.readFRSword(FRS_RECORDID_ACCELEROMETER, 3) & 0xFFFF; //Get word 3, lower 16 bits
  float accel_power = myIMU.qToFloat(accelerometer_power, 10); //Q point is 10 for power
  Serial.print("Accelerometer power: ");
  Serial.print(accel_power, 5);
  Serial.println(" (mA)");
}

void interrupt_handler() {  
   quatReady = true;
}

void bno_setup(TwoWire *wire, uint8_t imuIntPin) {
  delay(100);   //  Wait for BNO to boot
  Wire.flush(); // Reset I2C
      
  if (!myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire, imuIntPin)) {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));    
  }

  Wire.setClock(400000); // Increase I2C data rate to 400kHz    
  attachInterrupt(digitalPinToInterrupt(imuIntPin), interrupt_handler, FALLING);
  interrupts();

  //bno_calibrate();    

  //myIMU.tareNow(true);
  //myIMU.saveTare();  
  myIMU.enableRotationVector(50); // Send data update every 50ms
  //myIMU.enableGeoMagneticRotationVector(50); // Send data update every 50ms
  //bno_printStats();  
  //myIMU.enableMagnetometer(100);
  //myIMU.enableGyro(100);
  //myIMU.enableAccelerometer(100);
  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw, x, y, z"));  
  myIMU.getReadings();
}

void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}

void bno_getData() {  
  if (!quatReady)
    return;
  uint16_t datatype = myIMU.getReadings();
  if (datatype != SENSOR_REPORTID_ROTATION_VECTOR) 
    return;
  
    //myIMU.clearTare();
  
    quatReady = false;
    float roll = (myIMU.getRoll()) * 180.0 / PI;   // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI;     // Convert yaw / heading to degrees*/

    /*float mx = myIMU.getMagX();
    float my = myIMU.getMagY();
    float mz = myIMU.getMagZ();
    byte magAccuracy = myIMU.getMagAccuracy();
    float gx = myIMU.getGyroX();
    float gy = myIMU.getGyroY();
    float gz = myIMU.getGyroZ();
    float ax = myIMU.getAccelX();
    float ay = myIMU.getAccelY();
    float az = myIMU.getAccelZ();*/
  
    Serial.print("Euler: ");
    Serial.print(roll, 1);
    Serial.print(F(","));
    Serial.print(pitch, 1);
    Serial.print(F(","));
    Serial.print(yaw, 1);
    
    /*Serial.print("Gyro: ");
    Serial.print(gx, 2);
    Serial.print(F(","));
    Serial.print(gy, 2);
    Serial.print(F(","));
    Serial.print(gz, 2);
    Serial.print(F(","));*/

    /*Serial.print(" Accelerometer: ");
    Serial.print(ax, 2);
    Serial.print(F(","));
    Serial.print(ay, 2);
    Serial.print(F(","));
    Serial.print(az, 2);
    Serial.print(F(","));*/
    
    /*Serial.print(" Mag: ");
    Serial.print(mx, 2);
    Serial.print(F(", "));
    Serial.print(my, 2);
    Serial.print(F(", "));
    Serial.print(mz, 2);
    Serial.print(F(", "));
        
    printAccuracyLevel(magAccuracy);*/
    Serial.println();
  //}
}

void bno_calibrate() {
  Serial.println(F("Magnetometer Calirate start"));
  myIMU.calibrateMagnetometer();    
}

int secondsCounter = 0;

void bno_calibrate_magnetometer_loop() {
  /*secondsCounter++;
  Serial.print("Seconds ");*/
  Serial.println(secondsCounter);
  if (secondsCounter == 60) {
    Serial.println("Saving calibration");
    myIMU.saveCalibration();
    myIMU.requestCalibrationStatus();
    int counter = 100;
    while(1) {
      if(--counter == 0) break;
        if(myIMU.dataAvailable() == true) {          
          if(myIMU.calibrationComplete() == true) {
          Serial.println("Calibration data successfully stored");
          delay(1000);
          break;
        }
      }
      delay(100);
    }
    if(counter == 0) {
      Serial.println("Calibration data failed to store. Please try again.");
    }
  } else if(myIMU.dataAvailable()) {          
    float mx = myIMU.getMagX();
    float my = myIMU.getMagY();
    float mz = myIMU.getMagZ();
    byte magAccuracy = myIMU.getMagAccuracy();
    
    Serial.print(" Mag: ");
    Serial.print(mx, 2);
    Serial.print(F(", "));
    Serial.print(my, 2);
    Serial.print(F(", "));
    Serial.print(mz, 2);
    Serial.print(F(", "));
        
    printAccuracyLevel(magAccuracy);
    Serial.println();
  }  
}
