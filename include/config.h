#pragma once

#define APP_VERSION "4.0.0";
#define TWAI_TX_GPIO 5
#define TWAI_RX_GPIO 4
#define ENGINE_POWER_GPIO 13
#define CAMERA_POWER_GPIO 14
#define IMU_INT_GPIO 17
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_SPEED 400000U

#define CAN_SHOULDER_QUATERNION 0x20
#define CAN_SHOULDER_ACCELEROMETER 0x21
#define CAN_SHOULDER_GYROSCOPE 0x22
#define CAN_SHOULDER_ACCURACY 0x23
#define CAN_SHOULDER_SET_YZ_DEGREE 0x24
