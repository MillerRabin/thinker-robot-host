#include "twai.h"

bool TWAI::sendData(uint32_t id, uint8_t *data) {
  CanFrame obdFrame = {0};
  obdFrame.identifier = id;
  obdFrame.data_length_code = 8;
  obdFrame.extd = 0;
  memcpy(obdFrame.data, data, 8);
  if (sendSemaphore == NULL) {    
    return false;
  }

  if (xSemaphoreTake(sendSemaphore,
                     pdMS_TO_TICKS(CAN_SEND_WAIT_SEMAPHORE)) != pdTRUE) {
    // Serial.println("Can't obtain sendSemaphore for SendData");
    return false;
  };
  canSendMap[id % CAN_MAX_MESSAGE_ID] = obdFrame;
  xSemaphoreGive(sendSemaphore);
  return true;
}

twai_status_t TWAI::begin(TwaiSpeed twaiSpeed) {
  speed = twaiSpeed;

  if (callbackTaskHandle == NULL) {
    if (xTaskCreate(callbackTask, "TWAI::callbackTask", 2048, this, 2,
                    &callbackTaskHandle) != pdPASS) {
      printf("Failed to create TWAI::callbackTask\n");
      end();
      return TWAI_CALLBACK_TASK_CREATION_FAILED;
    }
  }
  vTaskSuspend(callbackTaskHandle);

  if (receiveTaskHandle == NULL) {
    if (xTaskCreate(receiveTask, "TWAI::receive", 2048, this, 5,
                    &receiveTaskHandle) != pdPASS) {
      printf("Failed to create TWAI::receiveTask\n");
      end();
      return TWAI_RECEIVE_TASK_CREATION_FAILED;
    }
  }
  vTaskSuspend(receiveTaskHandle);

  if (sendTaskHandle == NULL) {
    if (xTaskCreate(sendTask, "TWAI::send", 2048, this, 2, &sendTaskHandle) !=
        pdPASS) {
      printf("Failed to create TWAI::sendTask\n");
      end();
      return TWAI_SEND_TASK_CREATION_FAILED;
    }
  }

  vTaskSuspend(sendTaskHandle);

  if (initTaskHandle == NULL) {
    if (xTaskCreate(initTask, "TWAI::initTask", 2048, this, 2,
                    &initTaskHandle) != pdPASS) {
      printf("Failed to create TWAI::initTask\n");
      end();
      return TWAI_INIT_TASK_CREATION_FAILED;
    }
  }    
  vTaskResume(initTaskHandle);

  return TWAI_SUCCESS;
}

void TWAI::receiveTask(void *instance) {  
  TWAI *twai = static_cast<TWAI *>(instance);
  if (twai->receiveSemaphore == NULL) {
    vTaskSuspend(NULL);    
  }

  printf("TWAI::Receive task started\n");
  CanFrame rxFrame;
  while (true) {
    if (twai->readFrame(&rxFrame, portMAX_DELAY)) {
      if (xSemaphoreTake(twai->receiveSemaphore, pdMS_TO_TICKS(1)) == pdTRUE) {        
        twai->canReceiveMap[rxFrame.identifier % CAN_MAX_MESSAGE_ID] = rxFrame;
        xSemaphoreGive(twai->receiveSemaphore);
      }
    } else{
      if(!twai->isBusAlive()) {        
        twai->stopReceiver();
      }
    }
  }
}

void TWAI::stopReceiver() {
  printf("TWAI bus is not alive. Attempting to reinitialize from Receiver\n");
  vTaskSuspend(callbackTaskHandle);  
  vTaskSuspend(sendTaskHandle);  
  vTaskResume(initTaskHandle);
  vTaskSuspend(receiveTaskHandle);
}

void TWAI::stopSender() {
  printf("TWAI bus is not alive. Attempting to reinitialize from Sender\n");
  vTaskSuspend(callbackTaskHandle);  
  vTaskSuspend(receiveTaskHandle);  
  vTaskResume(initTaskHandle);
  vTaskSuspend(sendTaskHandle);
}

bool TWAI::isBusAlive() {
  twai_status_info_t statusInfo;
  if (twai_get_status_info(&statusInfo) == ESP_OK) {
    return statusInfo.state == TWAI_STATE_RUNNING;
  }
  return false;
}

void TWAI::initTask(void *instance) {
  printf("Twai initTask started\n");
  TWAI *twai = static_cast<TWAI *>(instance);

  while (true) {
    twai_status_t status = twai->init();
    if (status == TWAI_SUCCESS) {
      vTaskResume(twai->callbackTaskHandle);
      vTaskResume(twai->receiveTaskHandle);
      vTaskResume(twai->sendTaskHandle);
      vTaskSuspend(NULL);
    } else {
      printf("TWAI initialization failed with status: %d. Retrying...\n", status);
    }
    vTaskDelay(pdMS_TO_TICKS(TWAI_INITIALIZE_LOOP_DELAY));
  }  
}

void TWAI::sendTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  if (twai->sendSemaphore == NULL) {
    vTaskSuspend(NULL);
  }
  printf("TWAI::Send task started\n");
  uint32_t id = 0;
  while (true) {    
    if (xSemaphoreTake(twai->sendSemaphore,
                       pdMS_TO_TICKS(CAN_SEND_WAIT_SEMAPHORE)) != pdTRUE) {
      printf("Can't obtain sendSemaphore in sendTask\n");
      continue;
    }

    auto frame = twai->canSendMap[id];
    twai->canSendMap[id].data_length_code = 0;
    xSemaphoreGive(twai->sendSemaphore);
    if (frame.data_length_code == 0) {
      id++;
      if (id >= CAN_MAX_MESSAGE_ID) {
        id = 0;
      }
      continue;
    }

    if (twai->writeFrame(&frame)) {
      twai->errorCallback(frame, CAN_SUCCESS);
      id++;
      if (id >= CAN_MAX_MESSAGE_ID) {
        id = 0;
      }
    }  else {
      twai->errorCallback(frame, CAN_SEND_ERROR);
      if (!twai->isBusAlive()) {      
        twai->stopSender();
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(CAN_SEND_LOOP_WAIT));
  }
}

void TWAI::callbackTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  if (twai->receiveSemaphore == NULL) {
    vTaskSuspend(NULL);
  }

  TickType_t lastWakeTime = xTaskGetTickCount();

  CanFrame localFrame[CAN_RECEIVE_BUFFER_SIZE];

  uint frameIndex = 0;
  uint id = 0;

  while (true) {
    if (xSemaphoreTake(twai->receiveSemaphore, pdMS_TO_TICKS(CAN_RECEIVE_WAIT_TIMEOUT)) == pdTRUE) {
      while (id < CAN_MAX_MESSAGE_ID && frameIndex < CAN_RECEIVE_BUFFER_SIZE) {
        auto frame = twai->canReceiveMap[id];
        if (frame.data_length_code != 0) {
          twai->canReceiveMap[id].data_length_code = 0;
          localFrame[frameIndex++] = frame;
        }
        id++;
      }

      if (id >= CAN_MAX_MESSAGE_ID) {
        id = 0;
      }

      xSemaphoreGive(twai->receiveSemaphore);
    }

    for (uint i = 0; i < frameIndex; i++) {
      twai->callback(localFrame[i]);
    }

    frameIndex = 0;
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_RECEIVE_LOOP_TIMEOUT));
  }
}

twai_status_t TWAI::end() {
  auto sStatus = twai_stop();
  bool success = sStatus == ESP_OK || sStatus == ESP_ERR_INVALID_STATE;
  if (!success) {
    printf("TWAI::end() Failed to stop driver 0x%x\n", sStatus);
    return TWAI_STOP_FAILED;
  }

  sStatus = twai_driver_uninstall();
   if (sStatus != ESP_OK) {
    printf("TWAI::end() Failed to uninstall driver 0x%x\n", sStatus);
    return TWAI_UNISTALL_FAILED;
  }
  return TWAI_SUCCESS;
}

twai_status_t TWAI::init() {
  twai_status_t cStatus = end();
  bool success = cStatus == TWAI_SUCCESS || cStatus == TWAI_STOP_FAILED || cStatus == TWAI_UNISTALL_FAILED;
  if (!success) {
    printf("Failed to end driver, status: %d\n", cStatus);
    return cStatus;
  }

  if (sendSemaphore == NULL) {
    sendSemaphore = xSemaphoreCreateMutex();
    if (sendSemaphore == NULL) {
      printf("sendSemaphore allocation failed!\n");
      return TWAI_SEND_SEMAPHORE_ALLOCATION_FAILED;
    }
  }

  if (receiveSemaphore == NULL) {
    receiveSemaphore = xSemaphoreCreateMutex();
    if (receiveSemaphore == NULL) {
      printf("receiveSemaphore allocation failed!\n");
      return TWAI_RECEIVE_SEMAPHORE_ALLOCATION_FAILED;
    }
  }
  
  twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                    .tx_io = (gpio_num_t)txPin,
                                    .rx_io = (gpio_num_t)rxPin,
                                    .clkout_io = TWAI_IO_UNUSED,
                                    .bus_off_io = TWAI_IO_UNUSED,
                                    .tx_queue_len = txQueueSize,
                                    .rx_queue_len = rxQueueSize,
                                    .alerts_enabled = TWAI_ALERT_NONE,
                                    .clkout_divider = 0,
                                    .intr_flags = ESP_INTR_FLAG_LEVEL1};

  twai_timing_config_t t_config[TWAI_SPEED_SIZE] = {
      TWAI_TIMING_CONFIG_100KBITS(), TWAI_TIMING_CONFIG_125KBITS(),
      TWAI_TIMING_CONFIG_250KBITS(), TWAI_TIMING_CONFIG_500KBITS(),
      TWAI_TIMING_CONFIG_800KBITS(), TWAI_TIMING_CONFIG_1MBITS()};
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  auto tConfig = &t_config[speed];   
  auto iStatus = twai_driver_install(&g_config, tConfig, &f_config);
  
  if (iStatus != ESP_OK) {
    printf("Failed to install driver, status: 0x%x\n", iStatus);
  }
  bool iSuccess = iStatus == ESP_OK || iStatus == ESP_ERR_INVALID_STATE;
  if (!iSuccess) {
    printf("Failed to install driver\n");
    end();
    return TWAI_INSTALL_FAILED;
  }

  if (twai_start() != ESP_OK) {
    printf("Failed to start driver\n");
    end();
    return TWAI_START_FAILED;
  }
  
  printf("TWAI initialized successfully with speed %d\n", speed);
  return TWAI_SUCCESS;
}

TWAI::~TWAI() {  
  if (initTaskHandle) {
    vTaskDelete(initTaskHandle);
    initTaskHandle = NULL;
  }
  if (callbackTaskHandle) {
    vTaskDelete(callbackTaskHandle);
    callbackTaskHandle = NULL;
  }
  if (receiveTaskHandle) {
    vTaskDelete(receiveTaskHandle);
    receiveTaskHandle = NULL;
  }
  if (sendTaskHandle) {
    vTaskDelete(sendTaskHandle);
    sendTaskHandle = NULL;
  }
  if (sendSemaphore) {
    vSemaphoreDelete(sendSemaphore);
    sendSemaphore = NULL;
  }
  if (receiveSemaphore) {
    vSemaphoreDelete(receiveSemaphore);
    receiveSemaphore = NULL;
  }
  end();
}