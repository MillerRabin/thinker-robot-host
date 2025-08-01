/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.
*/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <SPI.h>

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B

//Platform specific configurations

//Registers
const byte CHANNEL_COMMAND = 0;
const byte CHANNEL_EXECUTABLE = 1;
const byte CHANNEL_CONTROL = 2;
const byte CHANNEL_REPORTS = 3;
const byte CHANNEL_WAKE_REPORTS = 4;
const byte CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_UNCALIBRATED_GYRO 0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

#define FRS_RECORDID_ORIENTATION 0x2D3E

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_NOW 0
#define TARE_PERSIST 1
#define TARE_SET_REORIENTATION 2

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

class BNO080
{
public:
	boolean begin(uint8_t deviceAddress = BNO080_DEFAULT_ADDRESS, TwoWire &wirePort = Wire, uint8_t intPin = 255); //By default use the default I2C addres, and use Wire port, and don't declare an INT pin
	boolean beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin, uint32_t spiPortSpeed = 3000000, SPIClass &spiPort = SPI);

	void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

	void softReset();	  //Try to reset the IMU via software
	bool hasReset(); //Returns true if the sensor has reported a reset. Reading this will unflag the reset.
	uint8_t resetReason(); //Query the IMU for the reason it last reset
	void modeOn();	  //Use the executable channel to turn the BNO on
	void modeSleep();	  //Use the executable channel to put the BNO to sleep

	float qToFloat(int16_t fixedPointValue, uint8_t qPoint); //Given a Q value, converts fixed point floating to regular floating point number

	boolean waitForI2C(); //Delay based polling for I2C traffic
	boolean waitForSPI(); //Delay based polling for INT pin to go low
	boolean receivePacket(void);
	boolean getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
	boolean sendPacket(uint8_t channelNumber, uint8_t dataLength);
	void printPacket(void); //Prints the current shtp header and data packets
	void printHeader(void); //Prints the current shtp header (only)

	void enableRotationVector(uint16_t timeBetweenReports);	
	void enableGeoMagneticRotationVector(uint16_t timeBetweenReports);
	void enableGameRotationVector(uint16_t timeBetweenReports);
	void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
	void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
	void enableAccelerometer(uint16_t timeBetweenReports);
	void enableLinearAccelerometer(uint16_t timeBetweenReports);
	void enableGravity(uint16_t timeBetweenReports);
	void enableGyro(uint16_t timeBetweenReports);
	void enableUncalibratedGyro(uint16_t timeBetweenReports);
	void enableMagnetometer(uint16_t timeBetweenReports);
	void enableTapDetector(uint16_t timeBetweenReports);
	void enableStepCounter(uint16_t timeBetweenReports);
	void enableStabilityClassifier(uint16_t timeBetweenReports);
	void enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);
	void enableRawAccelerometer(uint16_t timeBetweenReports);
	void enableRawGyro(uint16_t timeBetweenReports);
	void enableRawMagnetometer(uint16_t timeBetweenReports);
	void enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

	bool dataAvailable(void);
	uint16_t getReadings(void);
	uint16_t parseInputReport(void);   //Parse sensor readings out of report
	uint16_t parseCommandReport(void); //Parse command responses out of report

	void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);
	float getQuatI();
	float getQuatJ();
	float getQuatK();
	float getQuatReal();
	float getQuatRadianAccuracy();
	uint8_t getQuatAccuracy();

	void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	void getUncalibratedGyro(float &x, float &y, float &z, float &bx, float &by, float &bz, uint8_t &accuracy);
	float getUncalibratedGyroX();
	float getUncalibratedGyroY();
	float getUncalibratedGyroZ();
	float getUncalibratedGyroBiasX();
	float getUncalibratedGyroBiasY();
	float getUncalibratedGyroBiasZ();
	uint8_t getUncalibratedGyroAccuracy();


	void getFastGyro(float &x, float &y, float &z);
	float getFastGyroX();
	float getFastGyroY();
	float getFastGyroZ();

	void getMag(float &x, float &y, float &z, uint8_t &accuracy);
	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	void getGravity(float &x, float &y, float &z, uint8_t &accuracy);
	float getGravityX();
	float getGravityY();
	float getGravityZ();
	uint8_t getGravityAccuracy();

	void calibrateAccelerometer();
	void calibrateGyro();
	void calibrateMagnetometer();
	void calibratePlanarAccelerometer();
	void calibrateAll();
	void endCalibration();
	void saveCalibration();
	void requestCalibrationStatus(); //Sends command to get status
	boolean calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

	void tareNow(bool zAxis=false, uint8_t rotationVectorBasis=TARE_ROTATION_VECTOR);
	void saveTare();
	void clearTare();
	
	uint8_t getTapDetector();
	uint32_t getTimeStamp();
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();

	int16_t getRawAccelX();
	int16_t getRawAccelY();
	int16_t getRawAccelZ();

	int16_t getRawGyroX();
	int16_t getRawGyroY();
	int16_t getRawGyroZ();

	int16_t getRawMagX();
	int16_t getRawMagY();
	int16_t getRawMagZ();

	float getRoll();
	float getPitch();
	float getYaw();

	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
	void sendCommand(uint8_t command);
	void sendCalibrateCommand(uint8_t thingToCalibrate);
	void sendTareCommand(uint8_t command, uint8_t axis=TARE_AXIS_ALL, uint8_t rotationVectorBasis=TARE_ROTATION_VECTOR);
	bool writeFRSRecord(uint16_t frsType, uint32_t *dataWords, uint8_t wordCount);

	//Metadata functions
	int16_t getQ1(uint16_t recordID);
	int16_t getQ2(uint16_t recordID);
	int16_t getQ3(uint16_t recordID);
	float getResolution(uint16_t recordID);
	float getRange(uint16_t recordID);
	uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	uint8_t shtpData[MAX_PACKET_SIZE];
	uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;

private:
	//Variables
	TwoWire *_i2cPort;		//The generic connection to user's chosen I2C hardware
	uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

	Stream *_debugPort;			 //The stream to send debug messages to if enabled. Usually Serial.
	boolean _printDebug = false; //Flag to print debugging variables

	SPIClass *_spiPort;			 //The generic connection to user's chosen SPI hardware
	unsigned long _spiPortSpeed; //Optional user defined port speed
	uint8_t _cs;				 //Pins needed for SPI
	uint8_t _wake;
	uint8_t _int;
	uint8_t _rst;

	bool _hasReset = false;		// Keeps track of any Reset Complete packets we receive. 

	//These are the raw sensor values (without Q applied) pulled from the user requested Input Report	
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;	
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
	uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;
};