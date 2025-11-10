#ifndef IMU_SVC_HPP_
#define IMU_SVC_HPP_

/*
 * IMU Streaming Service
 *
 * Provides BLE interface for real-time 9-axis IMU data streaming
 *
 * BLE IMU Service (0xFF10):
 * - 0xFF11: IMU Data (READ/NOTIFY) - 9x float32 (Accel, Gyro, Mag)
 *
 * Data format: [accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ]
 * - Accelerometer: m/s² (unit 0x2713)
 * - Gyroscope: degrees/second (unit 0x2700 unitless)
 * - Magnetometer: µT (unit 0x2774 tesla, exponent -6)
 */

#include "blex.hpp"

template<typename Blex>
struct IMUServiceImpl {
    // Type aliases to reduce boilerplate code
    using Perms = typename Blex::template Permissions<>;

    // User Description text
    static constexpr char IMU_DESC_TEXT[] = "IMU: Accel(m/s^2) | Gyro(dps) | Mag(uT)";

    using ImuFormatDesc = blex_standard::descriptors::AggregateFormat<
        // Accelerometer: IEEE-754 32-bit float, m/s²
        blex_standard::descriptors::PresentationFormat<
            Blex::GattFormat::kFloat32,                     // IEEE-754 32-bit floating point
            0,                                              // Exponent: 10^0
            Blex::GattUnit::kMetrePerSecondSquared          // m/s²
        >,
        // Gyroscope: IEEE-754 32-bit float, rad/s (unitless per BLE spec - no standard unit for angular velocity)
        blex_standard::descriptors::PresentationFormat<
            Blex::GattFormat::kFloat32,                     // IEEE-754 32-bit floating point
            0,                                              // Exponent: 10^0
            Blex::GattUnit::kUnitless                       // Unitless (no BLE standard for angular velocity)
        >,
        // Magnetometer: IEEE-754 32-bit float, µT (microtesla)
        blex_standard::descriptors::PresentationFormat<
            Blex::GattFormat::kFloat32,                     // IEEE-754 32-bit floating point
            -6,                                             // Exponent: 10^-6 for micro
            Blex::GattUnit::kTesla                          // Tesla (with exponent -6 = µT)
        >
    >;

    // User Description Descriptor
    using IMUUserDesc = blex_standard::descriptors::UserDescription<IMU_DESC_TEXT>;

    // IMU Characteristic - 9 floats (36 bytes): accel(3) + gyro(3) + mag(3)
    using IMUChar = typename Blex::template Characteristic<
        float[9],
        0XFF11,
        typename Perms::AllowRead::AllowNotify,
        IMUUserDesc,
        ImuFormatDesc
    >;
};

// IMU Service Service Template (policy-agnostic)
// Instantiate with your chosen Blex policy: IMUService<blim>::Service
template<typename Blex, typename C = IMUServiceImpl<Blex> >
struct IMUService : C, Blex::template Service<
    0xFF10,
    typename C::IMUChar
> {
};
#endif //IMU_SVC_HPP_