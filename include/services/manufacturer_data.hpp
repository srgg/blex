#ifndef MANUFACTURER_DATA_HPP_
#define MANUFACTURER_DATA_HPP_
#include <cstdint>
#include <cstddef>

/*
 * BLIMCo Manufacturer Data
 * Provides compile-time manufacturer data for BLE advertising
 *
 * Format (7 bytes):
 * - Bytes 0-1: Company ID (0xFFFE = the last available unregistered company name to use 'unofficially')
 * - Byte 2:    Device Type (0x00 = BLE Test Device, 0x01 = IMU Streamer)
 * - Byte 3:    Hardware Version (0x10 = 1.0)
 * - Bytes 4-6: Firmware Version (Major.Minor.Patch)
 *
 * All values are automatically injected by version.py at build time.
 * To register a real Company ID, visit: https://www.bluetooth.com/specifications/assigned-numbers/
 */

// Fallback defaults (used only if version.py fails)
#ifndef FIRMWARE_VERSION
    #define FIRMWARE_VERSION "0.0.0-dev"
#endif

#ifndef HARDWARE_VERSION
    #define HARDWARE_VERSION "1.0"
#endif

#ifndef MANUFACTURER_ID
    #define MANUFACTURER_ID 0xFFFF // BLE test Company ID (0xFFFF for test/internal use)
#endif

// Compile-time version parsing utilities
namespace blimco {

constexpr uint8_t parse_version_component(const char* str, size_t& pos) {
    uint8_t value = 0;
    while (str[pos] >= '0' && str[pos] <= '9') {
        value = value * 10 + (str[pos] - '0');
        pos++;
    }
    if (str[pos] == '.' || str[pos] == '-') pos++;  // Skip delimiter
    return value;
}

struct Version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

constexpr Version parse_firmware_version(const char* ver) {
    size_t pos = 0;
    return Version{
        parse_version_component(ver, pos),
        parse_version_component(ver, pos),
        parse_version_component(ver, pos)
    };
}

constexpr uint8_t parse_hardware_version(const char* hw_ver) {
    // Parse "1.0" -> 0x10 (major in high nibble, minor in low nibble)
    size_t pos = 0;
    uint8_t major = parse_version_component(hw_ver, pos);
    const uint8_t minor = parse_version_component(hw_ver, pos);
    return (major << 4) | (minor & 0x0F);
}

// Device Types
enum class DeviceType : uint8_t {
    BLE_TEST_DEVICE = 0x00,  // Test peripheral with all supported BLE features
    IMU_STREAMER = 0x01,
    // Add more device types here as needed
};

// Manufacturer Data Builder
template<DeviceType Type = DeviceType::IMU_STREAMER>
struct ManufacturerData {
    static constexpr auto fw_ver = parse_firmware_version(FIRMWARE_VERSION);
    static constexpr auto hw_ver = parse_hardware_version(HARDWARE_VERSION);

    static constexpr uint8_t data[] = {
        static_cast<uint8_t>(MANUFACTURER_ID & 0xFF),          // Company ID low byte
        static_cast<uint8_t>((MANUFACTURER_ID >> 8) & 0xFF),   // Company ID high byte
        static_cast<uint8_t>(Type),                            // Device type
        hw_ver,                                                 // Hardware version
        fw_ver.major, fw_ver.minor, fw_ver.patch               // Firmware version
    };

    static constexpr size_t size = sizeof(data);
};

// Define storage for the static data array
template<DeviceType Type>
constexpr uint8_t ManufacturerData<Type>::data[];

} // namespace blimco

#endif // MANUFACTURER_DATA_HPP_