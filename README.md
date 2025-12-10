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
inline constexpr char deviceName[] = "MyBLEDevice";

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

- **[simple_server](examples/simple_server/)** - Minimal BLE server setup (start here!)
- **[basic_server](examples/basic_server/)** - Server with callbacks, descriptors, and OTA
- **[imu_streamer](examples/imu_streamer/)** - High-frequency sensor data streaming

Each example includes complete source code with inline documentation.

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
using MyServer = MyBlex::Server<deviceName,
    MyBlex::AdvertisementConfig<>
        ::WithTxPower<9>
        ::WithIntervals<100, 200>
>;
```

#### 3. Built-in Services

BLEX provides ready-to-use standard BLE services:

- **Device Information Service (0x180A)** - Manufacturer, model, firmware version, serial number
- **OTA/DFU Service** - Over-the-air firmware updates (Nordic DFU compatible)

See [Built-in Services Documentation](docs/built-in-services.md) for details.

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

### Connection Parameters

Request BLE connection parameter updates at runtime:

```cpp
// Request connection parameter update for a specific peer
MyServer::updateConnectionParams(conn_handle, 8, 15, 0, 4000);  // min/max interval (ms), latency, timeout

// Request connection parameter update for all connected peers
MyServer::updateAllConnectionParams(8, 15, 0, 4000);

// Restore default parameters (from ConnectionConfig)
MyServer::restoreDefaultConnectionParams(conn_handle);
```

**Available on:** `Server`, `Service`, and `Characteristic` types.

**Important:** These are *requests* to the central device. The central controls connection parameters and may ignore or modify the requested values. Behavior varies by platform:
- **Linux**: Generally honors peripheral requests
- **macOS/iOS**: Often ignores peripheral requests; central controls intervals
- **Android**: Usually honors requests within allowed ranges

### Characteristic Subscription Queries

Check which clients have subscribed to a characteristic's notifications or indications:

```cpp
// Count subscribers
uint8_t total = MyChar::getSubscriberCount();                                 // All subscribers
uint8_t notifyOnly = MyChar::getSubscriberCount(SubscriptionFilter::Notify);  // Notifications only

// Iterate with full connection details
MyChar::forEachSubscriber([](const SubscriptionInfo& info) {
    Serial.printf("Client %s: notify=%d, indicate=%d\n",
        info.connInfo.getAddress().toString().c_str(),
        info.isNotifySubscribed,
        info.isIndicateSubscribed);
});
```

**Filter options:** `SubscriptionFilter::Any` (default), `Notify`, `Indicate`

**`SubscriptionInfo` fields:**
- `connInfo` - Full `NimBLEConnInfo` (address, handle, MTU, security state, etc.)
- `isNotifySubscribed` / `isIndicateSubscribed` - Subscription flags

**Tip:** Return `false` from the callback to exit iteration early.

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
using MyServer = MyBlex::Server<deviceName,
    MyBlex::PassiveAdvService<DeviceInfoService<MyBlex>>,  // ADV_IND (all scanners)
    MyBlex::ActiveAdvService<BatteryService<MyBlex>>,      // SCAN_RSP (active scanners only)
    CustomService<MyBlex>                                  // Not advertised (GATT only)
>;
```

**When to use each:**
- **PassiveAdvService**: Critical services - in ADV_IND, visible to all scanners, limited space (~3-4 UUIDs)
- **ActiveAdvService**: Additional services - in SCAN_RSP, only visible to active scanners, 31 bytes available
- **Not advertised**: Background services not needed for discovery (OTA, diagnostics, configuration)

### High-Frequency Data Streaming

For sensor data streaming and real-time applications, BLEX provides optimized notification handling with lock-free performance optimizations. Choose between GATT Notifications (simple, widely compatible) or L2CAP CoC (high-throughput, low-latency).

See [High-Frequency Streaming Guide](docs/high-frequency-streaming.md) for detailed performance characteristics, optimization strategies, and implementation patterns.

### Binary Command Dispatcher

For implementing binary protocols over BLE characteristics, BLEX includes a zero-allocation command dispatcher:

```cpp
#include <blex/binary_command.hpp>

using MyDispatcher = blex_binary_command::Dispatcher<
    blex_binary_command::Command<0x01, [](const CreatePayload& p) { /* handle */ }>,
    blex_binary_command::Command<0x02, [](const QueryPayload& p) { /* handle */ }>,
    blex_binary_command::Fallback<[](uint8_t opcode, auto error) { /* handle unknown */ }>
>;

// In characteristic write handler:
MyDispatcher::dispatch(data, len);
```

**Features:**
- Opcode-based dispatch to typed handlers
- Compile-time payload size and alignment validation
- Zero heap allocation (static buffers sized from command definitions)

See `services/ota.hpp` for a complete usage example (Nordic DFU protocol implementation).

## API Reference

### AdvertisementConfig

```cpp
AdvertisementConfig<>
    ::WithTxPower<9>                    // TX power in dBm (-12 to 9)
    ::WithAppearance<kGenericComputer>  // BLE appearance (see BleAppearance)
    ::WithIntervals<100, 200>           // Min/max advertising intervals (0.625ms units)
    ::WithManufacturerData<data>        // Custom manufacturer data (see below)
    ::WithLongName<longName>            // Long/full device name for scan response (optional)
```

**Device naming:** The server's first template parameter is the short name (required), which appears in the advertising packet. Optionally, use `WithLongName<>` to provide an extended name in the 'active' scan response.

```cpp
// Short name only (used for both advertising and scan response)
inline constexpr char shortName[] = "MyDevice";
using MyServer = MyBlex::Server<shortName>;

// Separate short and long names
inline constexpr char shortName[] = "MyDev";
inline constexpr char longName[] = "My Device Full Name";
using MyServer = MyBlex::Server<shortName,
    MyBlex::AdvertisementConfig<>::WithLongName<longName>
>;
```

If the long name is omitted, the short name is used for both.

**Manufacturer Data** - two modes:
```cpp
// Raw mode: pass your own byte array
inline constexpr uint8_t myData[] = {0x34, 0x12, 0x01, 0x02};
::WithManufacturerData<myData>

// Builder mode: construct with helpers
::WithManufacturerData<>
    ::WithManufacturerId<0x1234>        // Required
    ::WithDeviceType<0x02>              // Optional TLV helper
    ::WithTLV<0x03, 0xAA, 0xBB>         // Optional custom TLV
```

### ConnectionConfig

Configure BLE connection parameters to balance throughput, latency, and power consumption:

```cpp
ConnectionConfig<>
    ::WithMTU<517>                      // Maximum MTU size (23-517)
    ::WithIntervals<12, 24>             // Min/max connection intervals (1.25ms units)
    ::WithLatency<0>                    // Slave latency (0-499)
    ::WithTimeout<4000>                 // Supervision timeout in ms (100-32000)
```

### SecurityConfig

`SecurityConfig` lets you define how your device handles BLE pairing, authentication, and encryption.
Each option is added using a fluent builder:

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

`ServerCallbacks` lets you attach handlers for important server-side BLE events.
These callbacks notify your application when a device connects, disconnects, or when link settings change.

```cpp
ServerCallbacks<>
    ::WithOnConnect<onConnect>                      // Called when a central connects
    ::WithOnDisconnect<onDisconnect>                // Called when a central disconnects
    ::WithOnMTUChange<onMTUChange>                  // MTU updated (e.g., 23 → larger)
    ::WithOnConnParamsUpdate<onConnParamsUpdate>    // Connection interval/latency update
```

**Callback signatures:**
```cpp
void onConnect(NimBLEConnInfo& conn);
void onDisconnect(NimBLEConnInfo& conn, int reason);
void onMTUChange(uint16_t mtu, NimBLEConnInfo& conn);
void onConnParamsUpdate(NimBLEConnInfo& conn);
```

### CharacteristicCallbacks

CharacteristicCallbacks lets you react to operations on a specific BLE characteristic.
Use these callbacks to control reads/writes, track subscription state, and monitor notification delivery.

```cpp
CharacteristicCallbacks<>
    ::WithOnRead<onRead>            // Called right before the value is sent to the client
    ::WithOnWrite<onWrite>          // Called when the client writes new data
    ::WithOnStatus<onStatus>        // Notification/indication delivery result
    ::WithOnSubscribe<onSubscribe>  // Client subscribes or unsubscribes to notifications
```

**Handler details:**
- **WithOnRead** — Triggered just before your characteristic's value is sent to the central device. Use this to update the value dynamically or perform access checks.
- **WithOnWrite** — Called when the central writes data into your characteristic. Ideal for parsing commands, validating input, or updating internal state.
- **WithOnStatus** — Reports the delivery status of notifications or indications—useful for reliability checks or flow control.
- **WithOnSubscribe** — Notifies you when the central subscribes or unsubscribes from notifications/indications. Use this to start/stop periodic updates or manage resource usage.

**Callback signatures:**
```cpp
void onRead(NimBLEConnInfo& conn);
void onWrite(const ValueType& value, NimBLEConnInfo& conn);  // or raw: (const uint8_t* data, size_t len, NimBLEConnInfo& conn)
void onStatus(NimBLECharacteristic* pChar, int code);
void onSubscribe(uint16_t subValue, NimBLEConnInfo& conn);
```

### setValue

Use `setValue` to update the value of a characteristic or descriptor from your code.
For characteristics, updating the value automatically sends a notification if a client is subscribed.

```cpp
MyChar::setValue(value);              // Set a typed value (auto-notifies if subscribed)
MyChar::setValue(data, size);         // Set value from a raw buffer
MyDescriptor::setValue(value);        // Set a typed value
MyDescriptor::setValue(data, size);   // Set from a raw buffer
```

**Returns:** `bool` — `false` if the characteristic or descriptor has not been registered.

**Supported input types**
- Primitive types (`int`, `float`, `bool`, etc.)
- `std::string`
- `Arduino String`
- `std::vector<T>`
- `std::array<T, N>`
- `const char*`
- Fixed-size C arrays (e.g., `uint8_t data[16]`)

**Note that:**
- Typed overloads handle serialization for you. For example, `setValue(42)` or `setValue(std::string("OK"))` “just works as is.”
- The raw-buffer overload (`setValue(data, size)`) is useful when you already have bytes prepared (e.g., a struct, sensor packet, or binary protocol).

## Architecture

BLEX follows a layered architecture with clean separation of concerns. Each layer has well-defined responsibilities and interfaces, enabling maintainability, testability, and potential backend portability.

See [Architecture Documentation](docs/architecture.md) for detailed descriptions of layers, design patterns, extension points, and performance considerations.

## Troubleshooting

### Compilation errors: "need 'typename' before dependent type"

**Symptom:** Build fails with errors like:
```
.pio/libdeps/.../blex/include/blex/core.hpp:121:18: error: need 'typename' before 'T::service_type'
because 'T' is a dependent scope
    using type = T::service_type;
                 ^
```

**Cause:** BLEX requires GCC 12+ which relaxes `typename` requirements (C++20 P0634R3). The default ESP32 toolchain (GCC 8.4) doesn't support this.

**Verify toolchain version:**
```bash
~/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-g++ --version 2>/dev/null || echo "Toolchain not found"
# Expected: xtensa-esp32s3-elf-g++ (crosstool-NG esp-12.2.0_20230208) 12.2.0
```

**Fix:** Add `platform_packages` to your `platformio.ini`:

```ini
[env]
platform = espressif32

; ESP32-S3
platform_packages = toolchain-xtensa-esp32s3 @ 12.2.0

; ESP32 (original)
; platform_packages = toolchain-xtensa-esp32 @ 12.2.0

; ESP32-C3 (RISC-V)
; platform_packages = toolchain-riscv32-esp @ 12.2.0
```

### OTA upload: "one of the arguments -d/--device -n/--name is required"

**Cause:** No target device specified for BLE OTA upload.

**Fix:** Pass device via environment variable:
```bash
PLATFORMIO_UPLOAD_FLAGS="-d aa:bb:cc:dd:ee:ff" pio run -e ota_server -t upload
# Or by name:
PLATFORMIO_UPLOAD_FLAGS="-n DeviceName" pio run -e ota_server -t upload
```

Or set a default in `platformio.ini`:
```ini
[env:ota_server]
upload_flags = -n "YourDeviceName"
```

See [OTA/DFU Service](docs/built-in-services.md#otadfu-service) for skip-rebuild and other OTA upload options.

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