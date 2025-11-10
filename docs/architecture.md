# BLEX Architecture

BLEX follows a layered architecture with clean separation of concerns, enabling maintainability, testability, and potential backend portability.

## Layer Diagram

```
┌─────────────────────────────────────────┐
│   Application Layer                     │
│   (Your BLE server configuration)       │
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
- Define BLE server using fluent builder API
- Implement custom services and characteristics
- Provide callback implementations
- Configure advertising, connections, security

**Example**:
```cpp
using MyDevice = MyBlex::Server<
    deviceName,
    deviceNameShort,
    MyBlex::AdvertisingConfig<>::WithTxPower<9>,
    MyBlex::ConnectionConfig<>::WithMTU<517>,
    MyBlex::SecurityConfig<>::WithPasskey<123456>,
    MyBlex::ServerCallbacks<>::WithOnConnect<onConnect>,
    MyCustomService<MyBlex>
>;
```

**Key Principle**: Declarative configuration with zero runtime overhead.

---

### BLEX API Layer (`blex.hpp`)

**Purpose**: Provide intuitive, type-safe, fluent builder API for BLE configuration.

**Responsibilities**:
- Re-export core types with simplified names
- Provide fluent builder interfaces
- Enforce configuration constraints via C++20 concepts
- Generate compile-time configuration structures

**Key Components**:
- `AdvertisingConfig<>` - Advertising parameters builder
- `ConnectionConfig<>` - Connection parameters builder
- `SecurityConfig<>` - Security/pairing configuration builder
- `ServerCallbacks<>` - Server event callbacks builder
- `Service<>`, `Characteristic<>` - Service/characteristic templates
- `Server<>` - Main server configuration template

**Example**:
```cpp
// Fluent builder chains compile to constexpr configuration
AdvertisingConfig<>
    ::WithTxPower<9>
    ::WithIntervals<100, 200>
    ::WithAppearance<kSensor>
```

**Key Principle**: All configuration resolved at compile-time - zero abstraction cost.

---

### Core Layer (`blex/core.hpp`)

**Purpose**: Core template metaprogramming engine and type system.

**Responsibilities**:
- Define configuration structures (AdvertisingConfig, ConnectionConfig, etc.)
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
struct AdvertisingConfig {
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
struct NoLock {
    void lock() const noexcept {}
    void unlock() const noexcept {}
};

// FreeRTOS mutex for multi-core thread safety
struct FreeRTOSLock {
    void lock() { xSemaphoreTakeRecursive(mutex, portMAX_DELAY); }
    void unlock() { xSemaphoreGiveRecursive(mutex); }
private:
    SemaphoreHandle_t mutex;
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
    using AdvConfig = typename Config::AdvertisingConfig;
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
struct AdvertisingConfig {
    template<uint8_t NewTxPower>
    using WithTxPower = AdvertisingConfig<NewTxPower, Appearance, ...>;

    template<uint16_t NewAppearance>
    using WithAppearance = AdvertisingConfig<TxPower, NewAppearance, ...>;
};

// Usage:
AdvertisingConfig<>
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
| Platform | Policy-dependent (0μs - 4μs) | Policy-dependent | New policies |
| Backend | NimBLE overhead (~40KB) | ~40KB + user services | New backends |

**Total BLEX overhead**: ~0-2KB beyond base NimBLE-Arduino stack.

---

## Summary

BLEX architecture achieves:
- ✅ Zero abstraction cost through compile-time configuration
- ✅ Type safety through C++20 concepts
- ✅ Flexibility through policy-based design
- ✅ Maintainability through layered separation
- ✅ Extensibility through well-defined interfaces

The architecture prioritizes embedded system constraints (minimal RAM, Flash, CPU) while providing modern C++ ergonomics and safety guarantees.