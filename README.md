# BLEX — simple, declarative BLE peripherals on NimBLE at zero cost.
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/framework-Arduino%20%7C%20ESP--IDF-green.svg)](https://platformio.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-20-orange.svg)](https://en.cppreference.com/w/cpp/20)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

BLEX is a modern C++20 (2a draft) layer on top of NimBLE-Arduino that makes BLE peripheral development type-safe, declarative, and easy to define. Template-driven design provides a fluent API with zero-cost abstractions—at compile time, everything reduces to direct NimBLE calls.

## Features
- **Zero Runtime Cost**: All BLEX configuration compiles to direct NimBLE calls
- **Fluent Builder Pattern**: Declarative, chainable, and expressive human-friendly configuration syntax
- **Type-Safe API**: C++20 concepts catch configuration errors at compile-time
- **Configurable Thread Safety**: Choose NoLock (zero overhead) or FreeRTOSLock (multicore-safe), or [implement your own](docs/thread-safety.md)
- **Out of the box ready-to-use services**: Device Info (0x180A), Nordic DFU-style OTA

## Quick Start

### Basic Example

```cpp
#include <Arduino.h>
#include <blex.hpp>

// Define your device
inline constexpr char deviceName[] = "MyDevice";
inline constexpr char deviceNameShort[] = "MyDev";

// Choose lock policy
using MyBlex = blexDefault;  // Thread-safe for multi-core
// using MyBlex = blex<NoLock>;  // Zero-overhead for single-core

// Configure BLE server with fluent API
using MyDevice = MyBlex::Server<
    deviceName,
    deviceNameShort,

    // Advertising Configuration
    MyBlex::AdvertisingConfig<>
        ::WithTxPower<9>
        ::WithAppearance<MyBlex::BleAppearance::kGenericComputer>
        ::WithIntervals<100, 200>,

    // Connection Configuration
    MyBlex::ConnectionConfig<>
        ::WithMTU<517>
        ::WithIntervals<12, 24>
        ::WithLatency<0>
        ::WithTimeout<4000>,

    // Security Configuration
    MyBlex::SecurityConfig<>
        ::WithIOCapabilities<DisplayYesNo>
        ::WithMITM<true>
        ::WithBonding<true>
        ::WithSecureConnections<true>
        ::WithPasskey<123456>,

    // Server Callbacks
    MyBlex::ServerCallbacks<>
        ::WithOnConnect<onConnect>
        ::WithOnDisconnect<onDisconnect>,

    // Services
    MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
>;

// Callback implementations
static void onConnect(NimBLEConnInfo& conn) {
    Serial.printf("Connected: %s\n", conn.getAddress().toString().c_str());
}

static void onDisconnect(NimBLEConnInfo& conn, int reason) {
    Serial.printf("Disconnected: reason=%d\n", reason);
    NimBLEDevice::startAdvertising();
}

void setup() {
    Serial.begin(115200);

    // Initialize BLE server
    if (!MyDevice::init()) {
        Serial.println("BLE initialization failed!");
        return;
    }

    Serial.println("BLE server started");
}

void loop() {
    delay(1000);
}
```

#### PlatformIO Installation
```ini
; platformio.ini
[env:esp32]
platform = espressif32@^6.0.0
framework = arduino
lib_deps =
    blex@^0.1.0
build_flags =
    -std=gnu++2a
    -fconcepts
build_unflags =
    -std=c++17
    -std=gnu++17
```

## Core Concepts

### 1. Expressive human-friendly compile-time configuration

All configuration is resolved at compile-time, resulting in zero runtime overhead:

```cpp
// This configuration produces optimized code with no runtime branching
MyBlex::AdvertisingConfig<>
    ::WithTxPower<9>
    ::WithIntervals<100, 200>
```

### 2. Lock Policies

Choose the appropriate thread-safety policy for your deployment:

```cpp
// Multi-core (ESP32 dual-core) - thread-safe with FreeRTOS mutexes
using MyBlex = blex<FreeRTOSLock>

// Single-core or pinned execution - zero overhead, no locking
using MyBlex = blex<NoLock>;
```

See [Thread Safety Documentation](docs/thread-safety.md) for detailed information on lock policies, custom implementations, and performance considerations.


### 3. Service Advertisement

Services can be advertised in ADV_IND (passive advertising), SCAN_RSP (scan response), both, or not advertised:

- **`PassiveAdvService<Svc>`**: Service UUID in ADV_IND (31 bytes, passively broadcast - visible to all scanners)
- **`ActiveAdvService<Svc>`**: Service UUID in SCAN_RSP (31 bytes, sent in response to SCAN_REQ - only visible to active scanners)
- **`BothAdvService<Svc>`**: Service UUID in both ADV_IND and SCAN_RSP
- **Not advertised**: Discoverable only via GATT service discovery after connection

```cpp
MyBlex::Server<
    /* ... config ... */,
    MyBlex::PassiveAdvService<DeviceInfoService<MyBlex>>,  // ADV_IND (all scanners)
    MyBlex::ActiveAdvService<BatteryService<MyBlex>>,      // SCAN_RSP (active scanners only)
    MyBlex::BothAdvService<HeartRateService<MyBlex>>,      // Both ADV_IND and SCAN_RSP
    CustomService<MyBlex>                                   // Not advertised (GATT only)
>
```

**When to use each:**
- **PassiveAdvService**: Critical services - in ADV_IND, visible to all scanners, limited space (~3-4 UUIDs)
- **ActiveAdvService**: Additional services - in SCAN_RSP, only visible to active scanners, 31 bytes available
- **BothAdvService**: Important services in both ADV_IND and SCAN_RSP
- **Not advertised**: Background services not needed for discovery (OTA, diagnostics, configuration)

## Built-in Services

### Device Information Service

Standard Bluetooth Device Information Service (0x180A):

```cpp
#include "services/device_info.hpp"

// Automatically uses build-time defines:
// - MANUFACTURER_NAME
// - SERIAL_NUMBER
// - HARDWARE_VERSION
// - MODEL_NUMBER
// - FIRMWARE_VERSION
// - SOFTWARE_REVISION

MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
```

### OTA/DFU Service

Nordic DFU-compatible over-the-air firmware update service:

```cpp
#include "services/ota.hpp"

MyBlex::PassiveAdvService<OtaService<MyBlex>>
```

**Features:**
- Resumable updates after power loss or disconnect
- CRC32 verification with incremental computation
- NVS-backed progress persistence (every 32KB)
- Packet receipt notifications (PRN) for reliability
- Compatible with Nordic DFU protocol

**Python Upload Tool:**
```bash
# Upload firmware via BLE
python lib/blex/tools/dfu_upload.py firmware.bin --device "MyDevice"
```

### Manufacturer Data

Compile-time manufacturer data for advertising packets:

```cpp
#include "services/manufacturer_data.hpp"

using namespace blimco;

MyBlex::AdvertisingConfig<>
    ::WithManufacturerData<ManufacturerData<DeviceType::IMU_STREAMER>::data>
```

**Format (7 bytes):**
- Bytes 0-1: Company ID (little-endian)
- Byte 2: Device Type
- Byte 3: Hardware Version (BCD: 0x10 = v1.0)
- Bytes 4-6: Firmware Version (Major.Minor.Patch)

## Custom Services

### Creating a Custom Service

```cpp
template<typename Blex>
struct MyCustomService : Blex::template Service<
    0xFF10,  // Service UUID
    // Characteristics...
    typename Blex::template Characteristic<
        uint32_t,                           // Value type
        0xFF11,                             // Characteristic UUID
        typename Blex::template Permissions<>
            ::AllowRead::AllowWrite::AllowNotify,
        typename Blex::template CharacteristicCallbacks<>
            ::WithOnWrite<onMyCharWrite>
    >
> {
    static void onMyCharWrite(const uint32_t& value) {
        Serial.printf("Received: %u\n", value);
    }
};

// Use in server configuration
MyBlex::Server<
    /* ... config ... */,
    MyCustomService<MyBlex>
>
```

### Advanced: Characteristic with Descriptors

```cpp
using namespace blex_standard;

template<typename Blex>
struct TemperatureSensor : Blex::template Service<
    0x1809,  // Health Thermometer Service
    typename Blex::template Characteristic<
        float,
        0x2A1C,  // Temperature Measurement
        typename Blex::template Permissions<>::AllowRead::AllowNotify,
        descriptors::UserDescription<"Temperature in Celsius">,
        descriptors::PresentationFormat<
            Blex::GattFormat::kFloat32,
            0,  // Exponent
            Blex::GattUnit::kCelsius
        >
    >
> {};
```

## Configuration Reference

### AdvertisingConfig

```cpp
AdvertisingConfig<>
    ::WithTxPower<9>                    // TX power in dBm (-12 to 9)
    ::WithAppearance<kGenericComputer>  // BLE appearance (see BleAppearance)
    ::WithIntervals<100, 200>           // Min/max advertising intervals (0.625ms units)
    ::WithManufacturerData<data>        // Custom manufacturer data
```

### ConnectionConfig

```cpp
ConnectionConfig<>
    ::WithMTU<517>                      // Maximum MTU size (23-517)
    ::WithIntervals<12, 24>             // Min/max connection intervals (1.25ms units)
    ::WithLatency<0>                    // Slave latency (0-499)
    ::WithTimeout<4000>                 // Supervision timeout in ms (100-32000)
```

### SecurityConfig

```cpp
SecurityConfig<>
    ::WithIOCapabilities<DisplayYesNo>  // I/O capabilities (see BleIOCapability)
    ::WithMITM<true>                    // Man-in-the-middle protection
    ::WithBonding<true>                 // Enable bonding/pairing
    ::WithSecureConnections<true>       // BLE Secure Connections
    ::WithPasskey<123456>               // Static 6-digit passkey
```

**I/O Capabilities:**
- `DisplayOnly` - Can only display passkey
- `DisplayYesNo` - Can display and confirm yes/no
- `KeyboardOnly` - Can only input passkey
- `NoInputNoOutput` - Just Works pairing
- `KeyboardDisplay` - Can both input and display

### ServerCallbacks

```cpp
ServerCallbacks<>
    ::WithOnConnect<onConnect>
    ::WithOnDisconnect<onDisconnect>
```

**Callback signatures:**
```cpp
void onConnect(NimBLEConnInfo& conn);
void onDisconnect(NimBLEConnInfo& conn, int reason);
```

## Architecture

BLEX follows a layered architecture with clean separation of concerns. Each layer has well-defined responsibilities and interfaces, enabling maintainability, testability, and potential backend portability.

See [Architecture Documentation](docs/architecture.md) for detailed layer descriptions, design patterns, extension points, and performance characteristics.

## Platform Support

| Platform | Architecture | Status |
|----------|--------------|--------|
| ESP32-S3 | Xtensa (240MHz) | ✅ Stable |

**Requirements:**
- PlatformIO espressif32 platform >= 6.0.0
- GCC >= 12.2.0 (for C++20 concepts)
- NimBLE-Arduino >= 2.3.6

## Examples

See the `examples/` directory for complete examples:

- **basic_server** - Minimal BLE server setup
- **custom_service** - Creating custom services
- **ota_update** - Over-the-air firmware updates
- **multi_service** - Multiple services with different characteristics
- **imu_streamer** - High-frequency sensor data streaming

## Logging

BLEX includes logging macros:

```cpp
// Set log level via build flags
build_flags =
    -DBLEX_LOG_LEVEL=BLEX_LOG_LEVEL_DEBUG

// Available macros:
BLEX_LOG_ERROR("Error: %d", code);
BLEX_LOG_WARN("Warning: %s", msg);
BLEX_LOG_INFO("Info: %s", msg);
BLEX_LOG_DEBUG("Debug: %d", value);
BLEX_LOG_DEBUG_BYTES("Packet: ", data, size);
```

## Contributing

Contributions are welcome! Please:

1. Follow the existing code style (C++20 (2a or later), template metaprogramming patterns)
2. Add tests for new features
3. Update documentation
4. Ensure backward compatibility

## License

MIT License - see [LICENSE](LICENSE) file for details

## Acknowledgments

- Built on top of [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- Inspired by modern C++ design patterns and zero-overhead abstractions
- Nordic DFU-like protocol for OTA updates

## Support

- **Issues**, **Feature requests**: [GitHub Issues](https://github.com/yourusername/blex/issues)

---

**Note**: BLEX requires C++20 compiler support. Ensure your PlatformIO configuration includes `-std=gnu++2a` and `-fconcepts` build flags.