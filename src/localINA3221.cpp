
#include "localINA3221.h"

INA3221 LocalINA3221::ina(INA3221_ADDR40_GND);
DetectorsCallback LocalINA3221::callback;
SemaphoreHandle_t LocalINA3221::loopMutex;
LocalINAData LocalINA3221::data;

bool LocalINA3221::begin(TwoWire& wire, DetectorsCallback callback) {
  LocalINA3221::callback = callback;
  LocalINA3221::loopMutex = xSemaphoreCreateMutex();
  if (!ina.begin(&wire)) {
    Serial.println("INA3221 is not found.");
    return false;
  }
  ina.reset();
  ina.setShuntRes(100, 100, 100);
  
  if (xTaskCreate(loop, "LocalINA3221::loop", 4096, this, 5, NULL) != pdPASS) {
    Serial.println("Failed to create loop task.");
    return false;
  }
  return true;
}

void LocalINA3221::loop(void* instance) {
  LocalINA3221* localINA = (LocalINA3221*)instance;
  while (true) {
    if (xSemaphoreTake(LocalINA3221::loopMutex, pdMS_TO_TICKS(10))) {
      localINA->data.powerCh1.set(localINA->ina.getVoltage(INA3221_CH1), localINA->ina.getCurrent(INA3221_CH1));
      localINA->data.powerCh2.set(localINA->ina.getVoltage(INA3221_CH2), localINA->ina.getCurrent(INA3221_CH2));
      localINA->data.powerCh3.set(localINA->ina.getVoltage(INA3221_CH3), localINA->ina.getCurrent(INA3221_CH3));
      xSemaphoreGive(LocalINA3221::loopMutex);
      localINA->callback(CAN_PLATFORM_CPU_POWER, localINA->data.serializeCPULine());
      localINA->callback(CAN_PLATFORM_ENGINES_POWER, localINA->data.serializeEnginesLine());      
    }    
    vTaskDelay(pdMS_TO_TICKS(50));  
  }
}

LocalINAData LocalINA3221::getLocalData() {
  LocalINAData localData;
  if (xSemaphoreTake(LocalINA3221::loopMutex, pdMS_TO_TICKS(10))) {
    localData = data;
    xSemaphoreGive(LocalINA3221::loopMutex);
  } else {
    //printf("INA3221 Can't obtain loopMutex semaphore lock");
  }
  return localData;
}

uint64_t LocalINAData::serializeCPULine() {
  uint64_t line = 0;
  line |= (uint64_t)powerCh2.serialize() << 32;
  line |= (uint64_t)powerCh3.serialize();
  return line;
}

uint64_t LocalINAData::serializeEnginesLine() {
  uint64_t line = 0;
  line |= powerCh1.serialize();  
  return line;
}