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
    sendBuffer[id % CAN_TX_BUFFER_SIZE] = obdFrame;     
    xSemaphoreGive(writeMutex);    
    return true;
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
    if (xTaskCreatePinnedToCore(callbackTask, "TWAI::callbackTask", 2048, this, 2, &callbackTaskHandle, 1) != pdPASS) {
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
    if (xTaskCreatePinnedToCore(watchdogTask, "TWAI::watchdogTask", 4096, this,
                                2, &watchdogTaskHandle, 1) != pdPASS) {
      printf("Failed to create TWAI::watchdogTask\n");
      end();
      return TWAI_WATCHDOG_TASK_CREATION_FAILED;
    }
  }

  requestReinit();
  return TWAI_SUCCESS;
}

void TWAI::receiveTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  CanFrame frame;
  uint32_t dropped = 0;
  uint32_t lastDropLog = 0;
  while (true) {
    // A line per dropped frame, at the ~3600 frames/s the modules send, kept this task printing and
    // starved the web server while an OTA upload stalled the frame consumer: task_wdt fired on
    // async_tcp (2026-11-06). Count them and report once a second instead.
    if (dropped > 0 && millis() - lastDropLog >= 1000) {
      printf("TWAI::receiveTask() receive buffer full, dropped %lu frames\n", (unsigned long)dropped);
      dropped = 0;
      lastDropLog = millis();
    }
    if (!twai->enterDriver()) {
      vTaskDelay(pdMS_TO_TICKS(CAN_DRIVER_RETRY_WAIT));
      continue;
    }
    // Short timeout: a reinstall waits for this call to return before it deletes the queue it blocks on
    auto status = twai->readFrame(&frame, pdMS_TO_TICKS(CAN_RECEIVE_DRIVER_WAIT));
    twai->leaveDriver();

    if (status == ESP_OK) {
      if (twai->receiveBuffer.push(frame)) {
        xEventGroupSetBits(twai->events, TWAI_EVENT_RX);
      } else {
        dropped++;
      }
    } else if (status != ESP_ERR_TIMEOUT) {
      vTaskDelay(pdMS_TO_TICKS(CAN_DRIVER_RETRY_WAIT));
    }
  }
}

// Call only between enterDriver() and leaveDriver()
bool TWAI::isBusAlive() {
  twai_status_info_t status;

  if (twai_get_status_info(&status) != ESP_OK)
    return false;

  // BUS_OFF, RECOVERING and STOPPED all refuse twai_transmit()
  return status.state == TWAI_STATE_RUNNING;
}

// Every twai_* call goes between enterDriver() and leaveDriver(): init() uninstalls the driver only once
// no task is inside it - deleting a queue a task is blocked on corrupts the heap.
bool TWAI::enterDriver() {
  if (reinitInProgress)
    return false;
  driverUsers++;
  if (reinitInProgress) {
    driverUsers--;
    return false;
  }
  return true;
}

void TWAI::leaveDriver() {
  driverUsers--;
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
  CanFrame localFrames[CAN_TX_BUFFER_SIZE];
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_SEND_LOOP_WAIT));

    // While the driver is being reinstalled (tens of ms) keep polling at the send period instead of
    // sleeping for seconds: sendBuffer holds the latest frame per id, so nothing stale piles up
    // meanwhile, and every platform frame (the modules' heartbeat) resumes at once.
    if (!twai->enterDriver())
      continue;

    if (twai->isBusAlive() &&
        xSemaphoreTake(twai->writeMutex, pdMS_TO_TICKS(CAN_SEND_WAIT_SEMAPHORE)) == pdTRUE) {
      memcpy(localFrames, twai->sendBuffer, sizeof(localFrames));
      memset(twai->sendBuffer, 0, sizeof(twai->sendBuffer));
      xSemaphoreGive(twai->writeMutex);

      for (uint i = 0; i < CAN_TX_BUFFER_SIZE; i++) {
        CanFrame frame = localFrames[i];
        if (frame.data_length_code == 0)
          continue;
        // Never block on a full TX queue: waiting 100 ms per frame stalls every other id behind it,
        // and the frame is superseded by the next period's value anyway.
        twai->errorCallback(frame, twai->writeFrame(&frame, 0) ? CAN_SUCCESS : CAN_SEND_ERROR);
      }
    }
    twai->leaveDriver();
  }  
}

// Bus-off is cleared by reinstalling the driver, the path this code has always used: in-place
// twai_initiate_recovery() crashed the IDF 4.4 driver on its first use here (assert tx_msg_count >= 0
// in the TX ISR, 2026-10-31). This ESP32 is the only node on the bus with CAN fault confinement
// (can2040 never goes bus-off), so it is the one that drops off when noise hits its frames - and all
// platform frames, the modules' heartbeat, stop until it is back.
void TWAI::watchdogTask(void *instance) {
  TWAI *twai = static_cast<TWAI *>(instance);
  uint32_t busOffCount = 0;
  uint32_t reportedBusErrors = 0;
  uint32_t maxTec = 0;
  TickType_t lastReport = 0;
  TickType_t lastReinit = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(CAN_WATCHDOG_WAIT));
    if (!twai->enterDriver())
      continue;
    twai_status_info_t status;
    bool installed = twai_get_status_info(&status) == ESP_OK;
    twai->leaveDriver();

    TickType_t now = xTaskGetTickCount();
    if (!installed) {
      // Only a failed init() leaves the driver uninstalled - retry, but not in a tight loop
      if (now - lastReinit >= pdMS_TO_TICKS(CAN_REINIT_WAIT)) {
        lastReinit = now;
        twai->requestReinit();
      }
      continue;
    }

    if (status.state != TWAI_STATE_RUNNING) {
      busOffCount++;
      printf("TWAI bus-off #%lu (state %d, bus errors %lu, TEC %lu), reinstalling\n",
             (unsigned long)busOffCount, status.state, (unsigned long)status.bus_error_count,
             (unsigned long)status.tx_error_counter);
      lastReinit = now;
      twai->requestReinit();
      continue;
    }

    if (status.tx_error_counter > maxTec)
      maxTec = status.tx_error_counter;
    if (status.bus_error_count != reportedBusErrors &&
        now - lastReport >= pdMS_TO_TICKS(CAN_ERROR_REPORT_INTERVAL)) {
      printf("TWAI bus errors %lu, tx failed %lu, rx missed %lu, TEC %lu (max %lu), REC %lu\n",
             (unsigned long)status.bus_error_count, (unsigned long)status.tx_failed_count,
             (unsigned long)status.rx_missed_count, (unsigned long)status.tx_error_counter,
             (unsigned long)maxTec, (unsigned long)status.rx_error_counter);
      reportedBusErrors = status.bus_error_count;
      maxTec = 0;
      lastReport = now;
    }
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
  TickType_t started = xTaskGetTickCount();
  reinitInProgress = true;
  while (driverUsers > 0)
    vTaskDelay(1);

  twai_status_t cStatus = end();
  vTaskDelay(pdMS_TO_TICKS(50));
  bool success = cStatus == TWAI_SUCCESS || cStatus == TWAI_STOP_FAILED ||
                 cStatus == TWAI_UNISTALL_FAILED;
  if (!success) {
    printf("Failed to end driver, status: %d\n", cStatus);
    reinitInProgress = false;
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
    reinitInProgress = false;
    return TWAI_INSTALL_FAILED;
  }

  if (twai_start() != ESP_OK) {
    printf("Failed to start driver\n");
    end();
    reinitInProgress = false;
    return TWAI_START_FAILED;
  }

  printf("TWAI initialized successfully with speed %d in %lu ms\n", speed,
         (unsigned long)((xTaskGetTickCount() - started) * portTICK_PERIOD_MS));
  reinitInProgress = false;
  return TWAI_SUCCESS;
}

void TWAI::requestReinit() {
  if (!reinitInProgress.exchange(true)) {
    xEventGroupSetBits(events, TWAI_EVENT_REINIT);
  }
}

TWAI::~TWAI() {
  if (writeMutex) {
    vSemaphoreDelete(writeMutex);
    writeMutex = NULL;
  }

  if (events == NULL) {
    vEventGroupDelete(events);
    events = NULL;
  }

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