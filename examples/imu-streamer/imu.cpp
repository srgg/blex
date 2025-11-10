#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#include "../../../../src/log.h"


// Sensor objects
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

// Pre-allocated array for BLE notification (9 floats)
float imuData[9]; // accelX,Y,Z; gyroX,Y,Z; magX,Y,Z

bool setup_imu() {
    // FeatherS3 I2C pins: SDA=GPIO8, SCL=GPIO9
    Wire.begin(8, 9);

    // I2C Scanner - diagnose what's on the bus
    BLIM_LOG_DEBUG("Scanning I2C bus...\n");
    uint8_t devices_found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            BLIM_LOG_DEBUG("  Found device at 0x%02X\n", addr);
            devices_found++;
        }
    }
    BLIM_LOG_DONE("Scan complete. Found %d device(s)\n", devices_found);

    if (!lsm6dsox.begin_I2C()) {
        BLIM_LOG_ERROR("Could not find LSM6DSOX!\n   Expected I2C address: 0x6A or 0x6B\n");
        return false;
    }
    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    if (!lis3mdl.begin_I2C()) {
        BLIM_LOG_ERROR("Could not find LIS3MDL!\n   Expected I2C address: 0x1C or 0x1E\n");
        return false;
    }
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    BLIM_LOG_DONE("IMU initialized\n");
    return true;
}

sensors_event_t accel_event, gyro_event, mag_event;
// bool read_imu(float* imu_data) {
bool read_imu(float (&imu_data)[9]) {
    // Read sensors via Unified Sensor API
    lsm6dsox.getAccelerometerSensor()->getEvent(&accel_event);
    lsm6dsox.getGyroSensor()->getEvent(&gyro_event);
    lis3mdl.getEvent(&mag_event);

    // Pack data into imuData[9]
    imu_data[0] = accel_event.acceleration.x;
    imu_data[1] = accel_event.acceleration.y;
    imu_data[2] = accel_event.acceleration.z;
    imu_data[3] = gyro_event.gyro.x;
    imu_data[4] = gyro_event.gyro.y;
    imu_data[5] = gyro_event.gyro.z;
    imu_data[6] = mag_event.magnetic.x;
    imu_data[7] = mag_event.magnetic.y;
    imu_data[8] = mag_event.magnetic.z;

    return true;
}