#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "config.h"
#include "detectors/detectors.h"

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define RX_TASK_PRIO 8
#define TX_TASK_PRIO 9
#define TX_GPIO_NUM gpio_num_t(CONFIG_TWAI_TX_GPIO_NUM)
#define RX_GPIO_NUM gpio_num_t(CONFIG_TWAI_RX_GPIO_NUM)

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static void twai_receive_task(void *arg)
{
  while (1)
  {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, portMAX_DELAY) != ESP_OK) {
      printf("bus error\n");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    if (rx_msg.identifier == CAN_SHOULDER_QUATERNION) {
      Quaternion quat;
      quat.deserialize(rx_msg.data);
      printf("Quaternion qx: %d, qy: %d, qz: %d, qw: %d\n", quat.i, quat.j, quat.k, quat.real);
    }

    if (rx_msg.identifier == CAN_SHOULDER_ACCELEROMETER) {
      Accelerometer acc;
      acc.deserialize(rx_msg.data);
      // printf("Accelerometer x: %d, y: %d, z: %d\n", acc.x, acc.y, acc.z);
    }

    if (rx_msg.identifier == CAN_SHOULDER_GYROSCOPE) {
      Gyroscope gyro;
      gyro.deserialize(rx_msg.data);
      // printf("Gyroscope x: %d, y: %d, z: %d\n", gyro.x, gyro.y, gyro.z);
    }

    if (rx_msg.identifier == CAN_SHOULDER_ACCURACY) {
      Accuracy acc;
      acc.deserialize(rx_msg.data);
      /*printf("Accuracy quat: %d, quatRad: %d, acc: %d, gyro: %d\n",
          acc.quaternionAccuracy,
          acc.quaternionRadAccuracy,
          acc.accelerometerAccuracy,
          acc.gyroscopeAccuracy
        );*/
    }    
  }
}

void twai_initialize(void){    
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_ERROR_CHECK(twai_start());    
  xTaskCreate(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL);
  //xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);  
}
