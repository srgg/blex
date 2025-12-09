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
 *
 * Thread Safety:
 * - IMU reads in loop() run on main Arduino thread
 * - BLE notifications triggered by setValue() are thread-safe (handled by NimBLE task)
 * - No explicit locking required: BLEX handles thread-safe characteristic updates
 * - I2C operations (Wire) are synchronous and run on the calling thread only
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <blex.hpp>

// ============================================================================
// IMU: Inertial Measurement Unit (Generic - Auto-detects hardware)
// ============================================================================

#include <Adafruit_SensorLab.h>

// Generic SensorLab - auto-detects IMU hardware
Adafruit_SensorLab lab;

// Generic sensor pointers (populated by SensorLab)
Adafruit_Sensor *accelerometer = nullptr;
Adafruit_Sensor *gyroscope = nullptr;
Adafruit_Sensor *magnetometer = nullptr;

bool setup_imu() {
    // FeatherS3 by Unexpected Maker: I2C pins: SDA=GPIO8, SCL=GPIO9
    Wire.begin(8, 9);

    Serial.println("SensorLab: Auto-detecting IMU hardware...");

    // Auto-detect and initialize accelerometer
    accelerometer = lab.getAccelerometer();
    if (!accelerometer) {
        Serial.println("Failed to detect accelerometer!");
        Serial.println("Supported: LSM6DS*, LSM9DS1, FXOS8700, ICM20*, MPU6050, etc.");
        return false;
    }

    // Auto-detect and initialize gyroscope
    gyroscope = lab.getGyroscope();
    if (!gyroscope) {
        Serial.println("Failed to detect gyroscope!");
        return false;
    }

    // Auto-detect and initialize magnetometer
    magnetometer = lab.getMagnetometer();
    if (!magnetometer) {
        Serial.println("Warning: No magnetometer detected (optional)");
        Serial.println("If you have a 9-DOF IMU, check I2C wiring.");
        // Don't fail - mag is optional
    }

    // Print detected sensor info
    sensor_t sensor_info;

    accelerometer->getSensor(&sensor_info);
    Serial.printf("✓ Accelerometer: %s (v%d)\n", sensor_info.name, sensor_info.version);
    Serial.printf("  Range: ±%.1f m/s², Resolution: %.4f m/s²\n",
                 sensor_info.max_value, sensor_info.resolution);

    gyroscope->getSensor(&sensor_info);
    Serial.printf("✓ Gyroscope: %s (v%d)\n", sensor_info.name, sensor_info.version);
    Serial.printf("  Range: ±%.1f rad/s, Resolution: %.4f rad/s\n",
                 sensor_info.max_value, sensor_info.resolution);

    if (magnetometer) {
        magnetometer->getSensor(&sensor_info);
        Serial.printf("✓ Magnetometer: %s (v%d)\n", sensor_info.name, sensor_info.version);
        Serial.printf("  Range: ±%.1f µT, Resolution: %.4f µT\n",
                     sensor_info.max_value, sensor_info.resolution);
    }

    Serial.println("IMU initialized successfully (generic SensorLab API)");
    return true;
}

bool read_imu(float (&imu_data)[9]) {
    static sensors_event_t accel_event, gyro_event, mag_event;

    // Read sensors via Unified Sensor API
    if (!accelerometer->getEvent(&accel_event)) {
        Serial.println("Accelerometer read failed!");
        return false;
    }

    if (!gyroscope->getEvent(&gyro_event)) {
        Serial.println("Gyroscope read failed!");
        return false;
    }

    // Magnetometer is optional
    if (magnetometer) {
        if (!magnetometer->getEvent(&mag_event)) {
            Serial.println("Magnetometer read failed!");
            // Zero out mag data, but don't fail
            memset(&imu_data[6], 0, 3 * sizeof(float));
        } else {
            memcpy(&imu_data[6], mag_event.magnetic.v, 3 * sizeof(float));
        }
    } else {
        // No magnetometer - zero out mag data
        memset(&imu_data[6], 0, 3 * sizeof(float));
    }

    // Copy accelerometer and gyroscope data
    memcpy(&imu_data[0], accel_event.acceleration.v, 3 * sizeof(float));
    memcpy(&imu_data[3], gyro_event.gyro.v,          3 * sizeof(float));

    return true;
}

// ============================================================================
// BLE
// ============================================================================

/// Device name variables with external linkage for template parameters
inline constexpr char shortName[] = "Blex-IMU";
inline constexpr char longName[] = "IMU Streamer";

/// User Description text
static constexpr char IMU_DESC_TEXT[] = "IMU: Accel(m/s^2) | Gyro(dps) | Mag(uT)";

using blx = blexDefault;

using IMUChar = blx::Characteristic<
    float[9],
    0XFF11,
    blx::Permissions<>
        ::AllowRead
        ::AllowNotify,

    blex_standard::descriptors::UserDescription<IMU_DESC_TEXT>,

    blex_standard::descriptors::AggregateFormat<
        // Accelerometer: IEEE-754 32-bit float, m/s²
        blex_standard::descriptors::PresentationFormat<
            blx::GattFormat::kFloat32,                      // IEEE-754 32-bit floating point
            0,                                              // Exponent: 10^0
            blx::GattUnit::kMetrePerSecondSquared>,         // m/s²

        // Gyroscope: IEEE-754 32-bit float, rad/s (unitless per BLE spec - no standard unit for angular velocity)
        blex_standard::descriptors::PresentationFormat<
            blx::GattFormat::kFloat32,                      // IEEE-754 32-bit floating point
            0,                                              // Exponent: 10^0
            blx::GattUnit::kUnitless>,                      // Unitless (no BLE standard for angular velocity)

        // Magnetometer: IEEE-754 32-bit float, µT (microtesla)
        blex_standard::descriptors::PresentationFormat<
            blx::GattFormat::kFloat32,                      // IEEE-754 32-bit floating point
            -6,                                             // Exponent: 10^-6 for micro
            blx::GattUnit::kTesla>                          // Tesla (with exponent -6 = µT)
    >
>;

using IMUService = blx::Service<
    0xFF10,
    IMUChar
>;

using ImuDevice = blx::Server<
    shortName,
    blx::AdvertisementConfig<>
        ::WithLongName<longName>
        ::WithTxPower<9>
        ::WithAppearance<blx::BleAppearance::kSensor>
        ::WithIntervals<120, 140>
        ::WithManufacturerData<>
            ::WithManufacturerId<0xfffe>
            ::WithDeviceType<0x02>,
    blx::ConnectionConfig<>
        ::WithMTU<517>
        ::WithIntervals<8, 10>        // 8-10ms - optimized for low-latency sensor streaming
        ::WithLatency<0>
        ::WithTimeout<4000>,          // 4s
    blx::SecurityConfig<>
        ::WithIOCapabilities<DisplayYesNo>
        ::WithMITM<true>
        ::WithBonding<true>
        ::WithSecureConnections<true>,
    blx::ActiveAdvService<DeviceInfoService<blx>>,
    IMUService,
    blx::PassiveAdvService<ota::OtaService<blx>>
>;

bool setup_ble() {
    return ImuDevice::startAllServices();
}

void update_imu(const float (&data)[9]) {
    IMUChar::setValue(data);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing IMU Streamer...");

    if (!setup_imu()) {
        Serial.println("Failed to initialize IMU");
        while (1) delay(1000);
    }

    if (!setup_ble()) {
        Serial.println("Failed to initialize BLE");
        while (1) delay(1000);
    }
    Serial.println("\nIMU Streamer initialized");
}

void loop() {
    static float imu_data[9];

    if (read_imu(imu_data)) {
        update_imu(imu_data);
    }

    delay(20); // 50Hz streaming
}