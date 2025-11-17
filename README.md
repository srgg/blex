# BLEX — Simplified type-safe BLE peripherals at no extra cost
[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/)
[![Framework](https://img.shields.io/badge/framework-Arduino%20%7C%20ESP--IDF-green.svg)](https://platformio.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-20-orange.svg)](https://en.cppreference.com/w/cpp/20)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Features
- **Zero Runtime Cost**: All BLEX configuration compiles to direct NimBLE calls
- **Fluent Builder Pattern**: Declarative, chainable, and expressive human-friendly configuration syntax
- **Type-Safe API**: C++20 concepts catch configuration errors at compile-time
- **Configurable Thread Safety**: Choose NoLock (zero overhead) or FreeRTOSLock (multicore-safe), or [implement your own](docs/thread-safety.md)
- **Ready-to-use services**: Device Info (0x180A), Nordic DFU-style OTA ([see docs](docs/built-in-services.md))

## Platform Support

| Platform | Architecture | Status |
|----------|--------------|--------|
| ESP32-S3 | Xtensa (240MHz) | ✅ Stable |

**Requirements:**
- PlatformIO espressif32 platform >= 6.0.0
- GCC >= 12.2.0 (for C++20 concepts)
- NimBLE-Arduino >= 2.3.6

## Installation

Add BLEX to your PlatformIO project:

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

## Quick Start

Create a simple BLE server with Device Information Service in just a few lines:

```cpp
#include <Arduino.h>
#include <blex.hpp>
#include <services/device_info.hpp>

// Define device name
constexpr const char* deviceName = "MyBLEDevice";

// Create BLE server with Device Info service
using MyBlex = blexDefault;  // Uses default settings (FreeRTOSLock)
using MyServer = MyBlex::Server<deviceName,
    MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
>;

void setup() {
    Serial.begin(115200);

    // Start BLE server and advertising
    MyServer::startAllServices();

    Serial.println("BLE Server started!");
    Serial.printf("Device name: %s\n", deviceName);
}

void loop() {
    // Your application logic here
    delay(1000);
}
```

**What this does:**
- Creates a BLE peripheral named "MyBLEDevice"
- Exposes Device Information Service (manufacturer, firmware version, etc.)
- Automatically injects device info from [`version.py`](docs/scripts.md) build script
- Starts advertising and accepts connections

**Next steps:** Check out the [examples](#examples) to learn more!

## Examples

See the `examples/` directory for complete working examples:

- **[basic_server](examples/basic_server/)** - Minimal BLE server setup (start here!)
- **[custom_service](examples/)** - Creating your own custom BLE services
- **[ota_update](examples/)** - Over-the-air firmware updates via BLE
- **[multi_service](examples/)** - Multiple services with different characteristics
- **[imu_streamer](examples/imu_streamer/)** - High-frequency sensor data streaming

Each example includes complete source code and detailed README with explanations.

## Understanding BLEX

### What is BLEX?

BLEX is a **declarative framework** for simplifying the creation of BLE peripheral devices on the ESP32. Instead of manually calling NimBLE APIs, you **declare** your device's structure using C++ templates, and BLEX compiles it into optimized NimBLE calls.

### Core Concepts

#### 1. Servers and Services

A **BLE Server** is your device that advertises and accepts connections. A **BLE Service** is a collection of related characteristics (data values) that central devices can read/write.

```cpp
// Server with built-in Device Info service
using MyServer = MyBlex::Server<deviceName,
    MyBlex::ActiveAdvService<DeviceInfoService<MyBlex>>
>;
```

#### 2. Compile-Time Configuration

All BLEX configuration happens at **compile-time**, resulting in zero runtime overhead:

```cpp
// This configuration produces optimized code with no runtime branching
MyBlex::Server<deviceName>
    ::WithAdvertising<>
        ::WithTxPower<9>
        ::WithAdvertisingInterval<100, 200>
```

#### 3. Built-in Services

BLEX provides ready-to-use standard BLE services:

- **Device Information Service (0x180A)** - Manufacturer, model, firmware version, serial number
- **OTA/DFU Service** - Over-the-air firmware updates (Nordic DFU compatible)
- **Manufacturer Data** - Custom advertising data

See [Built-in Services Documentation](docs/built-in-services.md) for details.

#### 4. Service Visibility

Control how services are advertised to central devices:

```cpp
MyBlex::PassiveAdvService<DeviceInfoService<MyBlex>>  // Advertised passively (all scanners)
MyBlex::ActiveAdvService<BatteryService<MyBlex>>      // Advertised on scan request
CustomService<MyBlex>                                 // Not advertised (GATT discovery only)
```

## Creating Custom Services

### Simple Custom Service

Create a custom service with a single characteristic:

```cpp
#include <blex.hpp>

template<typename Blex>
struct MyCustomService : Blex::template Service<
    0xFF10,  // Service UUID (use your own UUID)

    // Single characteristic: uint32_t value
    typename Blex::template Characteristic<
        uint32_t,                           // Value type
        0xFF11,                             // Characteristic UUID
        typename Blex::template Permissions<>
            ::AllowRead::AllowWrite::AllowNotify,
        typename Blex::template CharacteristicCallbacks<>
            ::WithOnWrite<onValueWrite>
    >
> {
    // Callback when central writes to characteristic
    static void onValueWrite(const uint32_t& value) {
        Serial.printf("Received value: %u\n", value);
    }
};

// Use in server
using MyServer = MyBlex::Server<deviceName,
    MyCustomService<MyBlex>
>;
```

### Advanced: Service with Descriptors

Add descriptors for richer characteristic metadata:

```cpp
using namespace blex_standard;

template<typename Blex>
struct TemperatureSensor : Blex::template Service<
    0x1809,  // Health Thermometer Service UUID

    typename Blex::template Characteristic<
        float,                              // Temperature value
        0x2A1C,                             // Temperature Measurement UUID
        typename Blex::template Permissions<>
            ::AllowRead::AllowNotify,

        // Add descriptors
        descriptors::UserDescription<"Temperature in Celsius">,
        descriptors::PresentationFormat<
            Blex::GattFormat::kFloat32,
            0,                              // Exponent
            Blex::GattUnit::kCelsius
        >
    >
> {};
```

## Runtime Control

### Service Management

Control services dynamically at runtime:

```cpp
// Start/stop individual services
MyService::start();      // Add service to server
MyService::stop();       // Remove service (can be re-added later)
MyService::isStarted();  // Check if service is active

// Start/stop all services
MyServer::startAllServices();  // Start all services + advertising
MyServer::stopAllServices();   // Stop all services
```

**Use cases:**
- Dynamic service activation based on device state
- Power management (disable unused services)
- Conditional feature exposure

### Advertising Control

Control advertising parameters at runtime:

```cpp
// Start/stop advertising
MyServer::startAdvertising();
MyServer::stopAdvertising();
MyServer::isAdvertising();

// Runtime parameter adjustment
MyServer::setTxPower(9);              // Change TX power (dBm)
MyServer::setAdvInterval(100, 200);   // Change intervals (ms)

// Reconfigure and restart advertising
MyServer::updateAdvertising();        // Stop, reconfigure, restart
```

**Note:** Runtime changes to advertising parameters override compile-time configuration.

### Device Naming

Configure device names for advertising and scan response:

```cpp
// Short name only (used for both advertising and scan response)
using MyServer = MyBlex::Server<&shortName>;

// Separate short and long names
using MyServer = MyBlex::Server<&shortName, &longName>;

// Or using fluent API
using MyServer = MyBlex::Server<&shortName>
    ::WithLongName<&longName>;
```

**BLE naming behavior:**
- **Short name**: Appears in advertising packet (limited space)
- **Long name**: Appears in scan response (more space)
- If only short name provided, it's used for both

## Advanced Topics

### Lock Policies

Choose the appropriate thread-safety policy for your deployment:

```cpp
// Multi-core (ESP32 dual-core) - thread-safe with FreeRTOS mutexes
using MyBlex = blex<FreeRTOSLock>;

// Single-core or pinned execution - zero overhead, no locking
using MyBlex = blex<NoLock>;
```

See [Thread Safety Documentation](docs/thread-safety.md) for detailed information on lock policies, custom implementations, and performance considerations.

### Service Advertisement Patterns

Control how services appear to BLE scanners:

**Advertisement Types:**
- **`PassiveAdvService<Svc>`**: Service UUID in ADV_IND (31 bytes, passively broadcast - visible to all scanners)
- **`ActiveAdvService<Svc>`**: Service UUID in SCAN_RSP (31 bytes, sent in response to SCAN_REQ - only visible to active scanners)
- **Not advertised**: Discoverable only via GATT service discovery after connection

```cpp
MyBlex::Server<
    /* ... config ... */,
    MyBlex::PassiveAdvService<DeviceInfoService<MyBlex>>,  // ADV_IND (all scanners)
    MyBlex::ActiveAdvService<BatteryService<MyBlex>>,      // SCAN_RSP (active scanners only)
    CustomService<MyBlex>                                  // Not advertised (GATT only)
>
```

**When to use each:**
- **PassiveAdvService**: Critical services - in ADV_IND, visible to all scanners, limited space (~3-4 UUIDs)
- **ActiveAdvService**: Additional services - in SCAN_RSP, only visible to active scanners, 31 bytes available
- **Not advertised**: Background services not needed for discovery (OTA, diagnostics, configuration)

### High-Frequency Data Streaming

For sensor data streaming and real-time applications, BLEX provides optimized notification handling with lock-free performance optimizations. Choose between GATT Notifications (simple, widely compatible) or L2CAP CoC (high-throughput, low-latency).

See [High-Frequency Streaming Guide](docs/high-frequency-streaming.md) for detailed performance characteristics, optimization strategies, and implementation patterns.

## API Reference

### AdvertisementConfig

```cpp
AdvertisementConfig<>
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

See [Architecture Documentation](docs/architecture.md) for detailed layer descriptions, design patterns, extension points, and performance considerations.

## Development

### Logging

BLEX includes logging macros for debugging:

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

### Contributing

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