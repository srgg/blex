/*
 * ESP32-S3 IMU Sensor BLE Device
 *
 * This firmware streams real-time 9-axis IMU data (accelerometer, gyroscope, magnetometer)
 * over BLE for motion capture and sensor calibration applications.
 *
 * Features:
 * - LSM6DSOX 6-axis IMU (accelerometer + gyroscope)
 * - LIS3MDL 3-axis magnetometer
 * - Custom BLE IMU Service with single 9-float characteristic
 * - Device Information Service with hardware/firmware details
 * - 50Hz continuous streaming (36 bytes/sample)
 * - Optimized advertising for passive/active scan discovery
 */

#include <Arduino.h>

#include "../../../../src/log.h"

// Forward declarations
extern bool setup_imu();
extern bool setup_ble();
extern bool read_imu(float (&data)[9]);
extern void update_imu(const float (&data)[9]);

void setup() {
    Serial.begin(115200);
    delay(1000);
    BLIM_LOG_DEBUG("Device Starting...\n");

    if (!setup_imu()) {
        BLIM_LOG_ERROR("Failed to initialize imu\n");
        while (1) delay(1000);
    }

    if (!setup_ble()) {
        BLIM_LOG_ERROR("Failed to initialize BLE\n");
        while (1) delay(1000);
    }

    BLIM_LOG_DONE("Device Started\n");
}

void loop() {
    static float imu_data[9];

    if (read_imu(imu_data)) {
        update_imu(imu_data);
    }

    delay(20); // 50Hz streaming
}