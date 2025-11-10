/**
 * @file blex.hpp
 * @brief Declarative, type-safe BLE peripherals on NimBLE at zero cost
 *
 * @details
 * Fluent builder API for BLE peripheral development with zero-cost abstractions.
 * Template-driven design provides an expressive, human-friendly configuration api
 * compiled to direct NimBLE calls - no runtime overhead, no dynamic dispatch.
 *
 * # Architecture
 *
 * ## Layered Design
 * - **API Layer**: Fluent builders and type-safe configuration (blex.hpp)
 * - **Core Layer**: Template metaprogramming engine (blex/core.hpp)
 * - **Platform Layer**: Lock policies and platform abstractions (blex/platform.hpp)
 * - **Backend Layer**: NimBLE integration (blex/nimble.hpp)
 *
 * ## Design Patterns
 * - **Fluent Builders**: Chainable configuration compiles to constexpr structures
 * - **CRTP**: Static polymorphism eliminates virtual function overhead
 * - **Policy-Based Design**: Configurable cross-cutting concerns (locking, logging)
 * - **Template Metaprogramming**: Compile-time configuration extraction and validation
 *
 * @see docs/architecture.md for detailed layer descriptions and extension points
 *
 * # Thread Safety
 *
 * ## Configurable Lock Policy
 * Choose synchronization strategy via template parameter:
 * - `blex<FreeRTOSLock>`: Multi-core safe (default on ESP32 dual-core)
 * - `blex<NoLock>`: Zero cost (single-core or externally synchronized)
 * - Custom: Implement your own lock policy
 *
 * ## Synchronization Guarantees
 * **Framework-synchronized** (automatic):
 * - `setValue()`: Thread-safe writes with consistent reads
 * - `init()`: Single-threaded initialization guard
 * - Metadata: Immutable after init (lock-free reads)
 *
 * **Application-synchronized** (user responsibility):
 * - Callbacks (`onRead`, `onWrite`, `onSubscribe`, `onStatus`) execute without locks
 * - Rationale: Avoid forced overhead, enable lock-free patterns, prevent deadlock
 *
 * @warning Callbacks must synchronize shared mutable state (globals, peripherals, task data)
 * @see docs/thread-safety.md for NimBLE guarantees, lock policies, and patterns
 *
 * # Example Usage
 *
 * @code{.cpp}
 * // Define framework with auto-detected lock policy
 * using Framework = blex<>;
 *
 * // Define characteristic with type-safe permissions
 * using TempChar = Framework::Characteristic<
 *     float,                                      // Value type
 *     0x2A6E,                                     // UUID (Temperature characteristic)
 *     Framework::Permissions<>::AllowRead::AllowNotify  // Permissions
 * >;
 *
 * // Implement callback (user provides synchronization if needed)
 * void on_read_temperature(float& value) {
 *     value = read_adc_temperature();  // Stack-local, no sync needed
 * }
 *
 * // Define service
 * using TempService = Framework::Service<
 *     0x181A,       // Environmental Sensing Service UUID
 *     TempChar
 * >;
 *
 * // Configure and initialize server
 * static constexpr char device_name[] = "TempSensor";
 * using Server = Framework::Server<&device_name, &device_name, TempService>;
 *
 * void setup() {
 *     Server::init();
 * }
 * @endcode
 *
 * @note Requires NimBLE-Arduino library for runtime BLE stack integration
 * @note Compiler requirements: GCC 12.2+ or Clang 14+ with -std=gnu++2a -fconcepts
 */

#ifndef BLEX_HPP_
#define BLEX_HPP_

#include "blex/platform.hpp"
#include "blex/core.hpp"
#include "blex/nimble.hpp"
#include "blex/standard.hpp"
#include "blex/log.h"

#include <services/device_info.hpp>
#include <services/ota.hpp>
#include <services/ots.hpp>

// ---------------------- Final Descriptor Types (Base + Backend) ----------------------

/**
 * @brief Generic descriptor (backend-integrated)
 */
template<typename T, auto UUID, typename Perms = Permissions<>::AllowRead, size_t MaxSize = sizeof(T)>
struct Descriptor : DescriptorBase<T, UUID, Perms, MaxSize> {
};

/**
 * @brief Const descriptor with compile-time value
 * @note Size calculated automatically from Value
 */
template<typename T, auto UUID, T Value, typename Perms = Permissions<>::AllowRead>
struct ConstDescriptor : DescriptorBase<T, UUID, Perms, blex_core::value_storage_size_v<T, Value>> {
    static constexpr T value = Value;

    static_assert(Perms::canRead &&
                  !Perms::canWrite &&
                  !Perms::canWriteNoResponse &&
                  !Perms::canNotify &&
                  !Perms::canIndicate,
                  "ConstDescriptor must be read-only (can have read security, but cannot write/notify/indicate)");
};

/**
 * @brief Presentation Format descriptor (GATT standard)
 */
template<uint8_t Format, int8_t Exponent, uint16_t Unit, uint8_t Namespace, uint16_t Description>
struct PresentationFormatDescriptor : PresentationFormatDescriptorBase<Format, Exponent, Unit, Namespace, Description> {
};

/**
 * @brief Aggregate Format descriptor (GATT standard)
 */
template<typename... PresentationFormatDescriptors>
struct AggregateFormatDescriptor : AggregateFormatDescriptorBase<PresentationFormatDescriptors...> {
};

// ---------------------- Final Characteristic Types (Global Scope) ----------------------

/**
 * @brief Const characteristic with compile-time value
 * @note Read-only, global scope for standard library use
 */
template<typename T, auto UUID, T Value, typename... Descriptors>
struct ConstCharacteristic : CharacteristicBase<T, UUID, Permissions<>::AllowRead, ConstCharacteristic<T, UUID, Value, Descriptors...>, Descriptors...> {
    using Backend = CharacteristicBackend<CharacteristicBase<T, UUID, Permissions<>::AllowRead, ConstCharacteristic<T, UUID, Value, Descriptors...>, Descriptors...>>;
    static constexpr T value = Value;
    static constexpr bool is_const_characteristic = true;
};

// ---------------------- BLEX Template Class ----------------------

template<
    template<typename> class LockPolicy = DefaultLock
>
struct blex {
    template<typename Tag>
    using lock_policy = LockPolicy<Tag>;

    // Re-export standard BLE types
    using BleAppearance = blex_standard::BleAppearance;
    using BleIOCapability = ::BleIOCapability;
    using GattFormat = blex_standard::GattFormat;
    using GattUnit = blex_standard::GattUnit;
    using PresentationFormatValue = blex_standard::PresentationFormatValue;

    // Re-export Permissions builder
    template<
        bool read = false,
        bool write = false,
        bool writeNoResponse = false,
        bool notify = false,
        bool indicate = false,
        uint8_t readSecurity = 0,
        uint8_t writeSecurity = 0,
        uint8_t writeNoRespSecurity = 0,
        uint8_t notifySecurity = 0,
        uint8_t indicateSecurity = 0
    >
    using Permissions = ::Permissions<read, write, writeNoResponse, notify, indicate, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;

    // Re-export config templates
    template<
        int8_t TxPower = 0,
        uint16_t IntervalMin = 100,
        uint16_t IntervalMax = 150,
        auto Appearance = BleAppearance::kUnknown,
        const uint8_t* ManufacturerData = nullptr,
        size_t ManufacturerDataLen = 0
    >
    using AdvertisingConfig = ::AdvertisingConfig<TxPower, IntervalMin, IntervalMax, Appearance, ManufacturerData, ManufacturerDataLen>;

    template<
        uint16_t MTU = 247,
        uint16_t ConnIntervalMinMs = 15,      // in milliseconds
        uint16_t ConnIntervalMaxMs = 15,      // in milliseconds
        uint16_t ConnLatency = 0,
        uint16_t SupervisionTimeoutMs = 4000  // in milliseconds
    >
    using ConnectionConfig = ::ConnectionConfig<MTU, ConnIntervalMinMs, ConnIntervalMaxMs, ConnLatency, SupervisionTimeoutMs>;

    template<
        BleIOCapability IOCapabilities = NoInputNoOutput,
        bool MITMProtection = false,
        bool Bonding = true,
        bool SecureConnections = true,
        uint32_t Passkey = 0
    >
    using SecurityConfig = ::SecurityConfig<IOCapabilities, MITMProtection, Bonding, SecureConnections, Passkey>;

    template<
        auto OnConnectCb = nullptr,
        auto OnDisconnectCb = nullptr,
        auto OnMTUChangeCb = nullptr
    >
    using ServerCallbacks = ::ServerCallbacks<OnConnectCb, OnDisconnectCb, OnMTUChangeCb>;

    template<
        auto OnReadCb = nullptr,
        auto OnWriteCb = nullptr,
        auto OnStatusCb = nullptr,
        auto OnSubscribeCb = nullptr
    >
    using CharacteristicCallbacks = ::CharacteristicCallbacks<OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>;

    // Advertising service wrappers
    template<typename Svc>
    struct PassiveAdvService {
        using service_type = Svc;
        static constexpr bool passive_adv = true;
        static constexpr bool active_adv = false;
    };

    template<typename Svc>
    struct ActiveAdvService {
        using service_type = Svc;
        static constexpr bool passive_adv = false;
        static constexpr bool active_adv = true;
    };

    template<typename Svc>
    struct BothAdvService {
        using service_type = Svc;
        static constexpr bool passive_adv = true;
        static constexpr bool active_adv = true;
    };

    // Re-export descriptors and characteristics
    template<typename T, auto UUID, T Value, typename Perms = Permissions<>::AllowRead>
    using ConstDescriptor = ::ConstDescriptor<T, UUID, Value, Perms>;

    template<uint8_t Format, int8_t Exponent, uint16_t Unit, uint8_t Namespace, uint16_t Description>
    using PresentationFormatDescriptor = ::PresentationFormatDescriptor<Format, Exponent, Unit, Namespace, Description>;

    template<typename... PresentationFormatDescriptors>
    using AggregateFormatDescriptor = ::AggregateFormatDescriptor<PresentationFormatDescriptors...>;

    template<typename T, auto UUID, typename Perms = Permissions<>::AllowRead, size_t MaxSize = sizeof(T)>
    using Descriptor = ::Descriptor<T, UUID, Perms, MaxSize>;

    // Re-export ConstCharacteristic from global scope
    template<typename T, auto UUID, T Value, typename... Descriptors>
    using ConstCharacteristic = ::ConstCharacteristic<T, UUID, Value, Descriptors...>;

    /**
     * @brief Final Characteristic type (Base + Backend)
     * @details Inherits metadata from CharacteristicBase, delegates operations to CharacteristicBackend
     */
    template<typename T, auto UUID, typename Perms, typename... Args>
    struct Characteristic : CharacteristicBase<T, UUID, Perms, Characteristic<T, UUID, Perms, Args...>, Args...> {
        // Base typedef inherited via CRTP from CharacteristicBase
        // Backend uses the parent CharacteristicBase type (not CRTP-resolved derived type)
        using Backend = CharacteristicBackend<CharacteristicBase<T, UUID, Perms, Characteristic<T, UUID, Perms, Args...>, Args...>>;
        
        // ---------------------- Backend Operations ----------------------

        /**
         * @brief Set value and notify (delegate to backend)
         */
        template<template<typename> class LP = LockPolicy>
        static void setValue(const T& newValue) {
            Backend::template setValue<LP>(newValue);
        }

        /**
         * @brief Set value from raw buffer (delegate to backend)
         */
        template<template<typename> class LP = LockPolicy>
        static void setValue(const uint8_t* data, size_t size) {
            Backend::template setValue<LP>(data, size);
        }
    };

    /**
     * @brief Final Service type (Base + Backend)
     * @details Inherits metadata from ServiceBase, delegates registration to ServiceBackend
     */
    template<auto UUID, typename... Chars>
    struct Service : ServiceBase<UUID, Service<UUID, Chars...>, Chars...> {
        using Backend = ServiceBackend<ServiceBase<UUID, Service<UUID, Chars...>, Chars...>>;
    };

    /**
     * @brief BLE Server (Base + Backend)
     * @details Main BLE server configuration. Inherits compile-time metadata from ServerBase via CRTP,
     * delegates runtime operations to ServerBackend.
     *
     * @tparam DeviceName Full device name (used in scan response)
     * @tparam ShortName Short name (used in advertising packet)
     * @tparam Args Variadic list accepting:
     *              - AdvertisingConfig<...> (optional, max 1)
     *              - ConnectionConfig<...> (optional, max 1)
     *              - SecurityConfig<...> (optional, max 1)
     *              - ServerCallbacks<...> (optional, max 1)
     *              - Service types (can be wrapped with PassiveAdvService/ActiveAdvService/BothAdvService)
     *
     * @par Example:
     * @code
     * using MyServer = blex<>::Server<
     *     &deviceName,
     *     &shortName,
     *     AdvertisingConfig<9, 120, 140>,         // TX power, adv intervals (ms)
     *     ConnectionConfig<247, 15, 15, 0, 4000>, // MTU, intervals (ms), latency, timeout (ms)
     *     SecurityConfig<>::WithIOCapabilities<DisplayYesNo>::WithMITM<true>,
     *     ServerCallbacks<>::WithOnConnect<myConnectHandler>,
     *     PassiveAdvService<MyService1>,           // Services
     *     ActiveAdvService<MyService2>
     * >;
     * @endcode
     *
     * @note Base typedef inherited via CRTP from ServerBase
     */
    template<
        const char* DeviceName,
        const char* ShortName,
        typename... Args
    >
    struct Server : ServerBase<DeviceName, ShortName, Server<DeviceName, ShortName, Args...>, Args...> {
        using Backend = ServerBackend<ServerBase<DeviceName, ShortName, Server<DeviceName, ShortName, Args...>, Args...>>;
        using ConnectionInfoType = typename Backend::ConnectionInfoType;

        // ---------------------- Lifecycle ----------------------

        [[nodiscard]]
        static bool init() {
            if (!Backend::init()) return false;

            // Register all services (uses blex internals, backend-agnostic)
            BLEX_LOG_DEBUG("[BLEX] init: registering services\n");
            register_all_services(typename Server::ServicesTuple{});
            BLEX_LOG_DEBUG("[BLEX] init: starting services\n");
            start_all_services(typename Server::ServicesTuple{});

            // Configure advertising
            BLEX_LOG_DEBUG("[BLEX] init: configuring advertising\n");
            configureAdvertising();

            // Start advertising
            Backend::startAdvertising();
            BLEX_LOG_DEBUG("[BLEX] server started (%s)\n", DeviceName);
            return true;
        }

        // ---------------------- Advertising Control ----------------------

        [[nodiscard]]
        static bool setTxPower(int8_t dbm) {
            return Backend::setTxPower(dbm);
        }

        [[nodiscard]]
        static bool setAdvInterval(uint16_t min_ms, uint16_t max_ms) {
            return Backend::setAdvInterval(min_ms, max_ms);
        }

        static void updateAdvertising() {
            BLEX_LOG_DEBUG("📡 Updating advertising configuration...\n");
            Backend::stopAdvertising();
            configureAdvertising();
            Backend::startAdvertising();
            BLEX_LOG_DEBUG("✓ Advertising updated and restarted\n");
        }

        // ---------------------- Connection Management ----------------------

        [[nodiscard]]
        static bool isConnected() {
            return Backend::isConnected();
        }

        [[nodiscard]]
        static uint16_t getConnectedCount() {
            return Backend::getConnectedCount();
        }

        static void disconnect(uint16_t conn_handle) {
            Backend::disconnect(conn_handle);
        }

        [[nodiscard]]
        static int8_t getRSSI(uint16_t conn_handle) {
            return Backend::getRSSI(conn_handle);
        }

    private:
        // ---------------------- Internal Service Registration ----------------------

        template<typename... Services>
        using ServicesPack = blex_core::ServicesPack<Services...>;

        template<typename ServiceOrWrapped>
        static void register_service() {
            using ActualService = typename blex_core::unwrap_service_impl<ServiceOrWrapped>::type;
            ActualService::validate();
            ActualService::Backend::template register_service<LockPolicy>(Backend::server);
        }

        template<typename ServiceOrWrapped>
        static void start_service() {
            using ActualService = typename blex_core::unwrap_service_impl<ServiceOrWrapped>::type;
            // Delegate to backend trait method (removes blex_nimble dependency)
            Backend::template startService<ActualService>();
        }

        template<typename... Services>
        static void register_all_services(ServicesPack<Services...>) {
            (register_service<Services>(), ...);
        }

        template<typename... Services>
        static void start_all_services(ServicesPack<Services...>) {
            (start_service<Services>(), ...);
        }

        /**
         * @brief Configure advertising (delegates to backend with passive/active services)
         */
        static void configureAdvertising() {
            Backend::template configureAdvertising<
                typename Server::PassiveServices,
                typename Server::ActiveServices
            >(DeviceName, ShortName);
        }
    };
};

// Convenience alias to use auto-detected lock policy
using blexDefault = blex<>;

#endif //BLEX_HPP_