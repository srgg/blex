/**
 * @file device_info.hpp
 * @brief Device Information Service (DIS) - Bluetooth SIG standard service 0x180A
 *
 * @details
 * Provides read-only device metadata via standard Bluetooth SIG Device Information
 * Service. All characteristic values are compile-time constants populated from
 * a build system (version.py) via preprocessor defines.
 *
 * # Service Structure
 * **UUID 0x180A** (Bluetooth SIG assigned)
 * - **Manufacturer Name** (0x2A29): READ - Company/organization name
 * - **Model Number** (0x2A24): READ - Device model/board identifier
 * - **Serial Number** (0x2A25): READ - Unique device serial
 * - **Hardware Revision** (0x2A27): READ - Hardware version string
 * - **Firmware Revision** (0x2A26): READ - Semantic version from Git tags
 * - **Software Revision** (0x2A28): READ - Git commit hash (with -dirty if modified)
 *
 * # Build Integration
 * Values injected by `version.py` at build time as `-D` preprocessor flags.
 *
 * @note All characteristics are const (compile-time) - no runtime modification
 * @note Policy-agnostic: Works with any Blex lock policy
 * @see Bluetooth SIG Device Information Service: https://www.bluetooth.com/specifications/specs/device-information-service-1-1/
 */

#ifndef DEVICE_INFO_SVC_HPP_
#define DEVICE_INFO_SVC_HPP_

#include "blex.hpp"

/// Fallback values used if version.py fails to inject build-time defines
#ifndef MANUFACTURER_NAME
    #define MANUFACTURER_NAME "unknown"
#endif

#ifndef SERIAL_NUMBER
    #define SERIAL_NUMBER "unknown"
#endif

#ifndef HARDWARE_VERSION
    #define HARDWARE_VERSION "0.0"
#endif

#ifndef MODEL_NUMBER
    #define MODEL_NUMBER "unknown"
#endif

#ifndef FIRMWARE_VERSION
    #define FIRMWARE_VERSION "0.0.0-dev"
#endif

#ifndef SOFTWARE_REVISION
    #define SOFTWARE_REVISION "unknown"
#endif

namespace detail {
    /// Bring blex_standard::chars into scope for unqualified access
    using namespace blex_standard::chars;

    /// @brief DeviceInfoServiceImpl - compile-time characteristic definitions
    template<typename Blex>
    struct DeviceInfoServiceImpl {
        static constexpr char manufacturer_name[] = MANUFACTURER_NAME;
        static constexpr char serial_number[] = SERIAL_NUMBER;
        static constexpr char hardware_version[] = HARDWARE_VERSION;
        static constexpr char model_number[] = MODEL_NUMBER;
        static constexpr char firmware_revision[] = FIRMWARE_VERSION;
        static constexpr char software_revision[] = SOFTWARE_REVISION;

        /// Type aliases to standard DIS characteristics
        using ManufacturerNameChar = ManufacturerName<manufacturer_name>;
        using ModelNumberChar = ModelNumber<model_number>;
        using SerialNumberChar = SerialNumber<serial_number>;
        using HardwareRevisionChar = HardwareRevision<hardware_version>;
        using FirmwareRevisionChar = FirmwareRevision<firmware_revision>;
        using SoftwareRevisionChar = SoftwareRevision<software_revision>;
    };
}

/**
 * @brief Device Information Service (DIS) template
 * @tparam Blex Lock policy type (e.g., blex<FreeRTOSLock> or blex<NoLock>)
 * @tparam C Implementation type (defaults to DeviceInfoServiceImpl<Blex>)
 */
template<typename Blex, typename C = detail::DeviceInfoServiceImpl<Blex> >
struct DeviceInfoService : C, Blex::template Service<
        0x180A,
        typename C::ManufacturerNameChar,
        typename C::ModelNumberChar,
        typename C::SerialNumberChar,
        typename C::HardwareRevisionChar,
        typename C::FirmwareRevisionChar,
        typename C::SoftwareRevisionChar
    >{
};

#endif