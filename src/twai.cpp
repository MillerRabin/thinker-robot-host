#include "twai.h"

bool TWAI::sendData(uint32_t id, uint8_t *data) {
  if (writeMutex == NULL)
    return false;
  CanFrame obdFrame = {0};
  obdFrame.identifier = id;
  obdFrame.data_length_code = 8;
  obdFrame.extd = 0;
  memcpy(obdFrame.data, data, 8);
  if (xSemaphoreTake(writeMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    auto res = sendBuffer.push(obdFrame);
    xSemaphoreGive(writeMutex);
    if (res) {
      xEventGroupSetBits(events, TWAI_EVENT_TX);
    }
    return res;
  } else {
    return false;
  }  
}

twai_status_t TWAI::begin(TwaiSpeed twaiSpeed) {
  speed = twaiSpeed;

  if (writeMutex == NULL) {
    writeMutex = xSemaphoreCreateMutex();
    if (writeMutex == NULL) {
      printf("Failed to create write mutex\n");
      return TWAI_WRITE_MUTEX_CREATION_FAILED;
    }
  }

  if (events == NULL) {
    events = xEventGroupCreate();
    if (events == NULL) {
      printf("Failed to create event group\n");
      return TWAI_EVENT_GROUP_CREATION_FAILED;
    }
  }

  if (callbackTaskHandle == NULL) {
    if (xTaskCreatePinnedToCore(callbackTask, "TWAI::callbackTask", 2048, this,
                                2, &callbackTaskHandle, 1) != pdPASS) {
      printf("Failed to create TWAI::callbackTask\n");
      end();
      return TWAI_CALLBACK_TASK_CREATION_FAILED;
    }
  }

  if (receiveTaskHandle == NULL) {
    if (xTaskCreatePinnedToCore(receiveTask, "TWAI::receive", 2048, this, 5,
                                &receiveTaskHandle, 1) != pdPASS) {
      printf("Failed to create TWAI::receiveTask\n");
      end();
      return TWAI_RECEIVE_TASK_CREATION_FAILED;
    }
  }

  if (sendTaskHandle == NULL) {
    if (xTaskCreate(sendTask, "TWAI::send", 4096, this, 5, &sendTaskHandle) !=
        pdPASS) {
      printf("Failed to create TWAI::sendTask\n");
      end();
      return TWAI_SEND_TASK_CREATION_FAILED;
    }
  }

  if (initTaskHandle == NULL) {
    if (xTaskCreatePinnedToCore(initTask, "TWAI::initTask", 2048, this, 2,
                                &initTaskHandle, 1) != pdPASS) {
      printf("Failed to create TWAI::initTask\n");
      end();
      return TWAI_INIT_TASK_CREATION_FAILED;
    }
  }

  if (watchdogTaskHandle == NULL) {
    if (xTaskCreatePinnedToCore(watchdogTask, "TWAI::watchdogTask", 2048, this,
                                2, &watchdogTaskHandle, 1) != pdPASS) {
      printf("Failed to create TWAI::watchdogTask\n");
      end();
      return TWAI_WATCHDOG_TASK_CREATION_FAILED;
    }
  }

  xTaskNotifyGive(initTaskHandle);
  return TWAI_SUCCESS;
}

void TWAI::receiveTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  CanFrame frame;
  while (true) {
    if (twai->readFrame(&frame, pdMS_TO_TICKS(100))) {
      if (twai->receiveBuffer.push(frame)) {
        xEventGroupSetBits(twai->events, TWAI_EVENT_RX);
      } else {
        printf("TWAI::receiveTask() receive buffer full, dropping frame\n");
      }
    } else {
      printf("Receive timeout\n");
    }
  }
}

bool TWAI::isBusAlive() {
  twai_status_info_t status;

  if (twai_get_status_info(&status) != ESP_OK)
    return false;

  if (status.state == TWAI_STATE_BUS_OFF)
    return false;

  if (status.state == TWAI_STATE_STOPPED) {
    printf("Bus is stopped\n");
    return false;
  }
    

  return true;
}

void TWAI::initTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  while (true) {
    xEventGroupWaitBits(twai->events, TWAI_EVENT_REINIT, pdTRUE, pdFALSE,
                        portMAX_DELAY);
    printf("Restarting TWAI\n");
    twai->init();
  }
}

void TWAI::sendTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  CanFrame frame;

  while (true) {
    xEventGroupWaitBits(twai->events, TWAI_EVENT_TX, pdTRUE, pdFALSE,
                        portMAX_DELAY);
    while (twai->sendBuffer.pop(&frame)) {      
      if (!twai->writeFrame(&frame, pdMS_TO_TICKS(100))) {
        twai->errorCallback(frame, CAN_SEND_ERROR);
        /*if (!twai->isBusAlive()) {
          printf("TWAI bus is not alive. Attempting to reinitialize from "
                 "SendTask\n");
          twai->requestReinit();
          break;
        }*/
      } 
      else {
        twai->errorCallback(frame, CAN_SUCCESS);
      }
    }
  }
}

void TWAI::watchdogTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  while (true) {
    if (!twai->isBusAlive()) {
      printf("TWAI bus is not alive. Attempting to reinitialize from "
             "Watchdog\n");
      twai->requestReinit();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TWAI::callbackTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);

  CanFrame frame;
  while (true) {
    xEventGroupWaitBits(twai->events, TWAI_EVENT_RX, pdTRUE, pdFALSE,
                        portMAX_DELAY);

    while (twai->receiveBuffer.pop(&frame)) {
      twai->callback(frame);
    }
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
  reinitInProgress = true;

  twai_status_t cStatus = end();
  vTaskDelay(pdMS_TO_TICKS(50));
  bool success = cStatus == TWAI_SUCCESS || cStatus == TWAI_STOP_FAILED ||
                 cStatus == TWAI_UNISTALL_FAILED;
  if (!success) {
    printf("Failed to end driver, status: %d\n", cStatus);
    return cStatus;
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
  reinitInProgress = false;
  return TWAI_SUCCESS;
}

void TWAI::requestReinit() {
  if (!reinitInProgress) {
    reinitInProgress = true;
    xEventGroupSetBits(events, TWAI_EVENT_REINIT);
  }
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
  end();
}