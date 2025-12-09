# BLEX Architecture

BLEX follows a layered architecture with clean separation of concerns, enabling maintainability, testability, and potential backend portability.

## Layer Diagram

```
┌─────────────────────────────────────────┐
│   Application Layer                     │
│   (Your BLE server configuration)       │
├─────────────────────────────────────────┤
│   Services Layer (services/*.hpp)       │
│   - DeviceInfo (0x180A)                 │
│   - OTA/DFU (0xFE59)                    │
│   - OTS (0x1825) [stub]                 │
│   (Optional, user includes)             │
├─────────────────────────────────────────┤
│   BLEX API Layer (blex.hpp)             │
│   - Fluent builders                     │
│   - Type-safe configuration             │
│   - C++20 concepts                      │
├─────────────────────────────────────────┤
│   Core Layer (blex/core.hpp)            │
│   - CRTP patterns                       │
│   - Template metaprogramming            │
│   - Trait-based design                  │
├─────────────────────────────────────────┤
│   Platform Layer (blex/platform.hpp)    │
│   - Lock policies                       │
│   - Platform abstractions               │
├─────────────────────────────────────────┤
│   Backend Layer (blex/nimble.hpp)       │
│   - NimBLE integration                  │
│   - Low-level BLE operations            │
└─────────────────────────────────────────┘
```

## Layer Descriptions

### Application Layer

**Purpose**: User-facing BLE server definitions and configurations.

**Responsibilities**:
- Define a BLE server using the fluent builder API
- Implement custom services and characteristics
- Provide callback implementations
- Configure advertising, connections, security

**Example**:
```cpp
using MyDevice = MyBlex::Server<
    deviceName,
    MyBlex::AdvertisementConfig<>::WithTxPower<9>::WithLongName<deviceNameLong>,
    MyBlex::ConnectionConfig<>::WithMTU<517>,
    MyBlex::SecurityConfig<>::WithPasskey<123456>,
    MyBlex::ServerCallbacks<>::WithOnConnect<onConnect>,
    MyCustomService<MyBlex>
>;
```

**Key Principle**: Declarative configuration with zero runtime overhead.

---

### Services Layer (`services/`)

**Purpose**: Optional pre-built BLE services for common use cases.

**Responsibilities**:
- Provide standard Bluetooth SIG services (Device Info, OTS)
- Implement vendor-specific services (OTA/DFU)
- Serve as reference implementations for custom services

**Available Services**:
- **Device Information Service** (`device_info.hpp`): Bluetooth SIG 0x180A
  - Manufacturer, model, serial number, firmware version, etc
  - Compile-time constants from a build system
  - ~100-200 bytes Flash

- **OTA/DFU Service** (`ota.hpp`): Nordic DFU-compatible 0xFE59
  - Resumable firmware updates with NVS persistence
  - CRC32 verification, packet receipt notifications
  - ~2-3KB Flash + `src/ota.cpp` implementation

- **Object Transfer Service** (`ots.hpp`): Bluetooth SIG 0x1825 [stub]

**Key Principle**: Opt-in services are included only if explicitly referenced by the application.

---

### BLEX API Layer (`blex.hpp`)

**Purpose**: Provide an intuitive, type-safe, fluent builder API for BLE configuration.

**Responsibilities**:
- Re-export core types with simplified names
- Provide fluent builder interfaces
- Enforce configuration constraints via C++20 concepts
- Generate compile-time configuration structures

**Key Components**:
- `AdvertisementConfig<>` - Advertising parameters builder
- `ConnectionConfig<>` - Connection parameters builder
- `SecurityConfig<>` - Security/pairing configuration builder
- `ServerCallbacks<>` - Server event callbacks builder
- `Service<>`, `Characteristic<>` - Service/characteristic templates
- `Server<>` - Main server configuration template

**Example**:
```cpp
// Fluent builder chains compile to constexpr configuration
AdvertisementConfig<>
    ::WithTxPower<9>
    ::WithIntervals<100, 200>
    ::WithAppearance<kSensor>
```

**Key Principle**: All configuration resolved at compile-time - zero abstraction cost.

---

### Core Layer (`blex/core.hpp`)

**Purpose**: Core template metaprogramming engine and type system.

**Responsibilities**:
- Define configuration structures (AdvertisementConfig, ConnectionConfig, etc.)
- Implement CRTP (Curiously Recurring Template Pattern) for type composition
- Provide trait-based type introspection
- Implement compile-time type validation via concepts

**Key Components**:
- Configuration structs with template parameters
- CRTP base classes for services and characteristics
- Type traits: `is_service`, `is_characteristic`, `is_callback`
- C++20 concepts: `ServiceWrapper`, `CharacteristicWrapper`, `CallbackWrapper`
- Template metaprogramming utilities for type extraction and validation

**Example**:
```cpp
template<
    uint8_t TxPower = 3,
    uint16_t Appearance = 0,
    uint16_t MinInterval = 160,
    uint16_t MaxInterval = 240,
    const uint8_t* ManufacturerData = nullptr,
    size_t ManufacturerDataSize = 0
>
struct AdvertisementConfig {
    static constexpr uint8_t tx_power = TxPower;
    // ... fluent builders ...
};
```

**Key Principle**: Compile-time type safety with zero runtime cost.

---

### Platform Layer (`blex/platform.hpp`)

**Purpose**: Abstract platform-specific concerns (threading, logging, etc.).

**Responsibilities**:
- Define lock policy abstractions
- Provide platform detection and configuration
- Abstract platform-specific types and APIs

**Key Components**:

#### Lock Policies
```cpp
// Zero-overhead policy for single-core/pinned execution
// Note: Actual implementation is templated on Tag type for per-resource locking
template<typename Tag = void>
struct NoLock {
    void lock() const noexcept {}
    void unlock() const noexcept {}
};

// FreeRTOS mutex for multi-core thread safety
// Note: Actual implementation uses static per-Tag mutex with lazy initialization
template<typename Tag = void>
struct FreeRTOSLock {
    void lock() const noexcept { /* acquire recursive mutex */ }
    void unlock() const noexcept { /* release recursive mutex */ }
};
```

#### Platform Detection
```cpp
#if CONFIG_FREERTOS_UNICORE
    #pragma message("BLEX: Single-core platform, consider NoLock policy")
#else
    #pragma message("BLEX: Multi-core platform, using FreeRTOSLock")
#endif
```

**Key Principle**: Configurable policies for different deployment scenarios. See [Thread Safety](thread-safety.md) for details.

---

### Backend Layer (`blex/nimble.hpp`)

**Purpose**: Translate BLEX configuration to NimBLE-Arduino API calls.

**Responsibilities**:
- Initialize NimBLE stack
- Apply compile-time configuration to runtime BLE stack
- Create and register services, characteristics, descriptors
- Wire up callbacks to NimBLE event handlers
- Manage BLE stack lifecycle

**Key Operations**:

#### Initialization
```cpp
template<typename Config>
static bool init() {
    // Extract configuration via template metaprogramming
    using AdvConfig = typename Config::AdvertisementConfig;
    using ConnConfig = typename Config::ConnectionConfig;
    using SecConfig = typename Config::SecurityConfig;

    // Initialize NimBLE
    NimBLEDevice::init(Config::device_name);

    // Apply configuration
    NimBLEDevice::setPower(AdvConfig::tx_power);
    NimBLEDevice::setMTU(ConnConfig::mtu);
    NimBLEDevice::setSecurityPasskey(SecConfig::passkey);

    // Register services (compile-time loop unrolling)
    registerServices<Config::Services...>();

    // Start advertising
    startAdvertising<AdvConfig>();
}
```

#### Service Registration
```cpp
template<typename Service>
static void registerService() {
    auto* pService = pServer->createService(Service::uuid);

    // Register characteristics (compile-time recursion)
    registerCharacteristics<Service, Service::Characteristics...>(pService);

    pService->start();
}
```

**Key Principle**: Direct translation from compile-time config to NimBLE calls - zero abstraction overhead.

---

## Design Patterns

### 1. Fluent Builder Pattern

**Purpose**: Provide readable, chainable configuration syntax.

**Implementation**:
```cpp
template</* ... params ... */>
struct AdvertisementConfig {
    template<uint8_t NewTxPower>
    using WithTxPower = AdvertisementConfig<NewTxPower, Appearance, ...>;

    template<uint16_t NewAppearance>
    using WithAppearance = AdvertisementConfig<TxPower, NewAppearance, ...>;
};

// Usage:
AdvertisementConfig<>
    ::WithTxPower<9>
    ::WithAppearance<kSensor>
```

**Benefits**: Type-safe, compile-time configuration with expressive syntax.

---

### 2. CRTP (Curiously Recurring Template Pattern)

**Purpose**: Enable static polymorphism without virtual function overhead.

**Implementation**:
```cpp
template<typename Derived, typename Blex>
struct ServiceBase {
    static NimBLEService* getService() {
        return Derived::service_instance;
    }
};

template<typename Blex>
struct MyService : ServiceBase<MyService<Blex>, Blex> {
    static inline NimBLEService* service_instance = nullptr;
};
```

**Benefits**: Zero-cost polymorphism, compile-time dispatch.

---

### 3. Policy-Based Design

**Purpose**: Allow configuration of cross-cutting concerns (locking, logging, etc.).

**Implementation**:
```cpp
template<typename LockPolicy = FreeRTOSLock>
struct blex {
    template<typename T>
    class ValueStorage {
        T value;
        mutable LockPolicy lock;
    public:
        void setValue(const T& v) {
            lock.lock();
            value = v;
            lock.unlock();
        }
    };
};
```

**Benefits**: Configurable behavior with zero overhead when not needed (NoLock policy).

---

### 4. Template Metaprogramming

**Purpose**: Extract and validate configuration at compile-time.

**Implementation**:
```cpp
// Extract specific config from variadic parameter pack
template<typename... Ts>
struct extract_config {
    using type = /* find matching type in Ts... */;
};

// Validate configuration with concepts
template<typename T>
concept ServiceWrapper = requires {
    typename T::ServiceType;
    { T::is_active_adv } -> std::convertible_to<bool>;
};
```

**Benefits**: Compile-time validation, zero runtime cost for configuration.

---

## Key Architectural Principles

### 1. Zero Abstraction Cost
All BLEX configuration code compiles away to direct NimBLE-Arduino calls. No runtime overhead for the abstraction layer itself.

### 2. Type Safety
C++20 concepts catch configuration errors at compile-time, preventing entire classes of runtime errors.

### 3. Separation of Concerns
Clear layer boundaries enable:
- Independent testing of layers
- Potential backend portability (e.g., adding BlueZ backend)
- Maintainability through isolated responsibilities

### 4. Compile-Time Configuration
All configuration resolved during compilation:
- No runtime branching for configuration
- Optimal code generation by compiler
- Minimal binary size and RAM usage

### 5. Policy-Based Flexibility
Pluggable policies (lock, logging) allow deployment-specific optimization without code duplication.

---

## Extension Points

### Adding a New Backend

To support a different BLE stack (e.g., BlueZ):

1. Implement backend layer: `blex/bluez.hpp`
2. Provide same template interface as `blex/nimble.hpp`
3. Translate BLEX config to BlueZ API calls
4. No changes needed to API, Core, or Platform layers

**Example**:
```cpp
// blex/bluez.hpp
namespace backend {
    template<typename Config>
    struct BlueZBackend {
        static bool init() {
            // Translate Config to BlueZ calls
        }
    };
}

// User code - unchanged
using MyBlex = blex<FreeRTOSLock, backend::BlueZBackend>;
```

### Adding Custom Lock Policy

See [Thread Safety Documentation](thread-safety.md) for implementing custom lock policies.

### Adding Custom Services

Services are first-class citizens in the architecture:

```cpp
template<typename Blex>
struct MyCustomService : Blex::template Service<
    0xFF10,  // UUID
    // Characteristics...
> {
    // Service-specific logic
};
```

No framework changes needed - services compose naturally with the existing architecture.

---

## Performance Characteristics

| Layer | Runtime Cost | Binary Size | Extensibility |
|-------|--------------|-------------|---------------|
| Application | User callbacks only | User code only | Full |
| API | Zero (compile-time) | Zero (templates) | Via Core layer |
| Core | Zero (compile-time) | Minimal template instantiation | New concepts/traits |
| Platform | Policy-dependent | Policy-dependent (see below) | New policies |
| Backend | Runtime BLE operations | ~1-3KB + NimBLE stack (~40KB) | New backends |

**Platform Layer Costs:**
- **NoLock**: ~1 byte RAM per unique Tag (heap allocation for empty class)
- **FreeRTOSLock**: Platform-dependent (ESP32-S3 @ 240MHz: ~80 bytes RAM per unique Tag, ~0.5-4μs lock/unlock)

**Backend Layer Costs:**
- **RAM**: ~10-20 bytes per service + ~10-20 bytes per characteristic (pointers, state)
- **Flash**:
  - Core BLEX backend: ~500-800 bytes (template instantiation, service registration)
  - Optional OTA service: ~2-3KB (only if included)
  - Optional DeviceInfo service: ~100-200 bytes (only if included)
- **Runtime**: Proportional to NimBLE stack operations (notify, setValue, etc.)

**Total BLEX overhead**:
- **Minimal config** (core only): ~500-800 bytes Flash, ~50-100 bytes RAM
- **With built-in services** (OTA + DeviceInfo): ~3-4KB Flash, ~100-400 bytes RAM

**Note**: BLEX is not zero-overhead abstraction - there is minimal RAM cost for runtime state (pointers, locks). However, configuration and type-safety are zero-cost (compile-time only). Built-in services are opt-in.

---

## Summary

BLEX architecture achieves:
- ✅ Zero abstraction cost through compile-time configuration
- ✅ Type safety through C++20 concepts
- ✅ Flexibility through policy-based design
- ✅ Maintainability through layered separation
- ✅ Extensibility through well-defined interfaces

The architecture prioritizes embedded system constraints (minimal RAM, Flash, CPU) while providing modern C++ ergonomics and safety guarantees.