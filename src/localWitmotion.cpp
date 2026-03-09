#include "localWitmotion.h"

uint32_t LocalWitmotion::c_uiBaud[8] = { 230400, 115200, 57600, 38400, 19200, 9600 };
volatile bool LocalWitmotion::dataAvailable = false;
LocalWitmotion *LocalWitmotion::instance = NULL;
SemaphoreHandle_t LocalWitmotion::loopMutex;

void LocalWitmotion::SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  instance->port.write(p_data, uiSize); 
}

void LocalWitmotion::readData() { 
  while (port.available()) {
    uint8_t c = port.read();
    WitSerialDataIn(c);
    lastReceiveTime = millis();
  }  
}

void LocalWitmotion::readDetectorTask(void *instance) {
  printf("Witmotion Sensor read task started\n");
  LocalWitmotion *wit = (LocalWitmotion *)instance;
  while (true) {
    wit->readData();
    if (!wit->isPresent()) {
      printf("Sensor lost\n");      
      vTaskResume(wit->initTaskHandle);
      vTaskSuspend(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void LocalWitmotion::setContent() {
  if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_PRESS | RSW_Q) != WIT_HAL_OK) {
    printf("Set RSW Error\n");
  } else {
    printf("Set content Mode Success\n");
  }  
}

void LocalWitmotion::set6AxisMode() {
  WitWriteReg(AXIS6, ALGRITHM6);
  Delayms(50);
  WitWriteReg(SAVE, 0x00);
  Delayms(100);
}

void LocalWitmotion::set9AxisMode() {
  WitWriteReg(AXIS6, ALGRITHM9);
  Delayms(50);
  WitWriteReg(SAVE, 0x00);
  Delayms(100);
}

void LocalWitmotion::setRefreshRate() {
  if (WitSetOutputRate(RRATE_200HZ) != WIT_HAL_OK) {
    printf("Failed to set rate\n");
  } else {
    printf("Output rate set to 200Hz. Saving settings\n");
    if (WitWriteReg(SAVE, 0x00) != WIT_HAL_OK) {
      printf("Failed to save settings\n");
    } else {
      printf("Settings saved\n");
    }
    Delayms(100);
  }
}

void LocalWitmotion::setBandwidth() {
  if (WitWriteReg(BANDWIDTH, BANDWIDTH_94HZ) != WIT_HAL_OK) {
    printf("Failed to set bandwidth\n");
  } else {
    printf("Bandwidth set to 94Hz\n");
    WitWriteReg(SAVE, 0x00);
    Delayms(100);
  }
}

void LocalWitmotion::init(void *instance) {
  printf("Witmotion Sensor scan started\n");
  LocalWitmotion *wit = (LocalWitmotion *)instance;

  while (true) {
    uint32_t scanResult = wit->autoScanSensor();
    if (scanResult != 0) {
      printf("Sensor found at baud rate: %lu\n", scanResult);
      wit->setBandwidth();
      wit->setRefreshRate();
      wit->set9AxisMode();
      wit->setContent();
      vTaskResume(wit->readDetectorTaskHandle);
      vTaskSuspend(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void LocalWitmotion::setBaudRate() {
  if (WitSetUartBaud(WIT_BAUD_230400) != WIT_HAL_OK) {
    printf("Baud change failed\n");
    return;
  }
  printf("230400 baud rate is set\n");
  Delayms(50);

  WitWriteReg(SAVE, 0x00);
  Delayms(100);
  
  port.end();
  delay(20);
  port.begin(230400, SERIAL_8N1, memsRxPin, memsTxPin);
}

void LocalWitmotion::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {    
  dataAvailable = true; 
  for (int i = 0; i < uiRegNum; i++) {
    if (uiReg > q3) {
      break;
    }
    switch (uiReg) {
    case AZ:
      instance->accelerometer.fromWitmotion(sReg[AX], sReg[AY], sReg[AZ]);
      instance->callback(CAN_PLATFORM_ACCELEROMETER,
                         instance->accelerometer.serialize());
      break;
    case GZ:
      instance->gyroscope.fromBNO(sReg[GX], sReg[GY], sReg[GZ]);
      instance->callback(CAN_PLATFORM_GYROSCOPE,
                         instance->gyroscope.serialize());
      break;
    case Yaw:
      break;
    case HeightH:    
      instance->barometer.set((int32_t)((sReg[HeightH] << 16) | sReg[HeightL]),
                              sReg[TEMP]);
      instance->callback(CAN_PLATFORM_BAROMETER,
                         instance->barometer.serialize());
      break;
    case q3:      
      
      instance->quaternion.fromWitmotion(sReg[q1], sReg[q2], sReg[q3],
                                         sReg[q0]);      
      instance->callback(CAN_PLATFORM_QUATERNION,
                         instance->quaternion.serialize());
      break;
    default:
      break;
    }
    uiReg++;
  }
  if (xSemaphoreTake(LocalWitmotion::loopMutex, pdMS_TO_TICKS(10))) {
    instance->data.rawQuaternion = instance->quaternion;    
    instance->data.rawAccelerometer = instance->accelerometer;
    instance->data.rawGyroscope = instance->gyroscope;
    instance->data.rawBarometer = instance->barometer;
    xSemaphoreGive(LocalWitmotion::loopMutex);
  }
}

void LocalWitmotion::Delayms(uint16_t usMs) { vTaskDelay(pdMS_TO_TICKS(usMs)); }

uint32_t LocalWitmotion::autoScanSensor() {
  int i, iRetry;  
  for (i = 0; i < 8; i++) {
    port.end();
    port.begin(c_uiBaud[i], SERIAL_8N1, memsRxPin, memsTxPin);
    port.flush();    
    iRetry = 2;
    do {
      dataAvailable = false;
      WitReadReg(AX, 3);
      Delayms(100);      
      readData();
      if (dataAvailable) {
        return c_uiBaud[i];
      }
      iRetry--;
    } while (iRetry);
  }
  return 0;
}

bool LocalWitmotion::begin() {
  BaseType_t initTaskStatus = xTaskCreate(readDetectorTask, "readDetectorTask", 4096, this, 5, &readDetectorTaskHandle);
  if (initTaskStatus != pdPASS) {
    printf("Failed to create readDetectorTask\n");
    return false;
  }
  xTaskCreate(init, "initTask", 4096, this, 5, &initTaskHandle);
  if (initTaskStatus != pdPASS) {
    printf("Failed to create initTask\n");
    return false;
  }
  return true;
}

LocalWitmotion::LocalWitmotion(HardwareSerial& port, 
                               const uint memsRxPin, const uint memsTxPin,
                               DetectorsCallback callback) : 
      port(port), 
      memsRxPin(memsRxPin), 
      memsTxPin(memsTxPin), 
      callback(callback) {
  port.setRxBufferSize(2048);
  LocalWitmotion::instance = this;  
  LocalWitmotion::loopMutex = xSemaphoreCreateMutex();  
  WitInit(WIT_PROTOCOL_NORMAL, 0xFF);  
  WitSerialWriteRegister(SensorUartSend);  
  WitRegisterCallBack(SensorDataUpdata);  
  WitDelayMsRegister(Delayms);  
}

LocalWitmotionData LocalWitmotion::getLocalData() {
  if (xSemaphoreTake(loopMutex, pdMS_TO_TICKS(IMU_WAIT_MUTEX))) {
    LocalWitmotionData localData = this->data;
    xSemaphoreGive(loopMutex);
    this->prevData = localData;   
    return localData;
  }
  return this->prevData;
}