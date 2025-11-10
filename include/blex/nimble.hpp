/**
 * @file nimble.hpp
 * @brief NimBLE backend - zero-cost compile-time to NimBLE runtime translation
 *
 * @details
 * Backend layer specializing trait templates (ServiceBackend, CharacteristicBackend,
 * DescriptorBackend, ServerBackend) for the NimBLE stack. Translates compile-time
 * type information into NimBLE API calls with no abstraction penalty.
 *
 * # Backend Specializations
 * - **ServiceBackend**: NimBLE service registration and lifecycle
 * - **CharacteristicBackend**: Value storage, callbacks, and thread-safe operations
 * - **DescriptorBackend**: Standard and custom descriptor registration
 * - **ServerBackend**: Advertising, connections, security configuration
 *
 * # Performance Optimizations
 * - **Read+Notify optimization**: Skips redundant onRead when value already notified
 * - **Direct notify path**: Notify-only characteristics bypass value storage
 * - **Static polymorphism**: Template-based callbacks eliminate virtual dispatch
 *
 * @note NimBLE-specific: Requires NimBLE-Arduino library
 * @see core.hpp for backend-agnostic trait definitions
 */

#ifndef BLEX_NIMBLE_HPP_
#define BLEX_NIMBLE_HPP_

#include "platform.hpp"
#include "core.hpp"
#include "standard.hpp"
#include "log.h"

// Detect if NimBLE is available
#ifdef NIMBLE_CPP_DEVICE_H_
    #define BLEX_NIMBLE_AVAILABLE
    #include <NimBLE2904.h>
    #include <NimBLE2905.h>
    // For ble_svc_gap_device_appearance_set()
    #if defined(CONFIG_NIMBLE_CPP_IDF)
        #include "host/ble_svc_gap.h"
    #else
        #include "nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h"
    #endif
#endif

// Forward declaration
template<template<typename> class LockPolicy>
struct blex;

namespace blex_nimble {

#ifdef BLEX_NIMBLE_AVAILABLE

// Helper to create NimBLEUUID
template<auto uuid>
static NimBLEUUID make_uuid() {
    using UUIDType = decltype(uuid);
    using U = std::remove_cv_t<std::remove_reference_t<UUIDType>>;
    if constexpr (std::is_integral_v<U>) {
        static_assert(sizeof(U) <= sizeof(uint16_t) ||
                     (std::is_signed_v<U> ? uuid >= 0 && uuid <= 0xFFFF : uuid <= 0xFFFF),
                     "UUID integer value must fit in uint16_t (0-65535)");
        return NimBLEUUID(static_cast<uint16_t>(uuid));
    } else {
        return NimBLEUUID(uuid);
    }
}

// Add service UUIDs to advertising
template<typename... Services>
static void add_service_uuids_impl(NimBLEAdvertisementData& advData, std::tuple<Services...>) {
    (advData.addServiceUUID(make_uuid<blex_core::unwrap_service_impl<Services>::type::uuid>()), ...);
}

#endif // BLEX_NIMBLE_AVAILABLE

} // namespace blex_nimble

// ---------------------- ServiceBackend Trait Specialization (NimBLE) ----------------------

#ifdef BLEX_NIMBLE_AVAILABLE

/**
 * @brief NimBLE service backend - registration and lifecycle
 */
template<auto UUID, typename Derived, typename... Chars>
struct ServiceBackend<ServiceBase<UUID, Derived, Chars...>> {
    using Base = ServiceBase<UUID, Derived, Chars...>;

    // NimBLE-specific storage - runtime service handle
    inline static NimBLEService* pService = nullptr;

    /**
     * @brief Register service and all its characteristics to NimBLE server
     * @tparam LockPolicy Lock implementation to use for characteristics
     * @param server NimBLE server pointer
     */
    template<template<typename> class LockPolicy>
    static void register_service(NimBLEServer* server) {
        // Create NimBLE service
        pService = server->createService(blex_nimble::make_uuid<UUID>());

        // Register all characteristics inline (blex<> will be complete when this is instantiated)
        register_all_chars<LockPolicy>(typename Base::chars_pack{});
    }

    /**
     * @brief Start the service (make it active)
     */
    static void start() {
        if (pService) {
            pService->start();
        }
    }

private:
    /**
     * @brief Helper to register all characteristics (directly to backends)
     */
    template<template<typename> class LockPolicy, typename... CharTypes>
    static void register_all_chars(blex_core::CharsPack<CharTypes...>) {
        // Call CharacteristicBackend for each characteristic (use Characteristic's Backend typedef)
        (CharTypes::Backend::template register_characteristic<LockPolicy>(pService), ...);
    }
};

#endif // BLEX_NIMBLE_AVAILABLE

// ---------------------- CharacteristicBackend Trait Specialization (NimBLE) ----------------------

#ifdef BLEX_NIMBLE_AVAILABLE

/**
 * @brief NimBLE backend implementation for Characteristic
 * @details Provides NimBLE-specific storage (pChar) and operations (setValue)
 */
template<typename T, auto UUID, typename Perms, typename Derived, typename... Args>
struct CharacteristicBackend<CharacteristicBase<T, UUID, Perms, Derived, Args...>> {
    using Base = CharacteristicBase<T, UUID, Perms, Derived, Args...>;

    // NimBLE-specific storage - runtime characteristic handle
    inline static NimBLECharacteristic* pChar = nullptr;

    // ---------------------- Nested Shim (NimBLE Callback Adapter) ----------------------

    /**
     * @brief NimBLE callback adapter (nested inside backend)
     * @tparam LockPolicy Lock implementation (FreeRTOSLock for multi-core, NoLock for single-core)
     */
    template<template<typename> class LockPolicy>
    struct Shim final : NimBLECharacteristicCallbacks {
        // Per-characteristic scope lock
        using guard_t = typename blex_sync::template ScopedLock<LockPolicy, Base>;

        // Determine if we should use read+notify optimization
        static constexpr bool use_read_notify_optimization =
            Base::perms_type::canNotify && Base::perms_type::canRead;

        // Empty container when optimization is disabled
        struct NoReadNotifyOptimization {};

        // Container for when optimization is enabled
        struct WithReadNotifyOptimization {
            std::atomic<int8_t> subscriber_count{0};
            std::atomic<bool> notified_value_valid{false};
        };

        inline static std::conditional_t<
            use_read_notify_optimization,
            WithReadNotifyOptimization,
            NoReadNotifyOptimization
        > read_notify_optimization;

    private:
        // Internal setValue without locking - accesses outer pChar directly
        static void setValue_unsafe(const typename Base::value_type& newValue) {
            using ValueT = typename Base::value_type;
            if (auto* p = pChar) {  // Access outer scope pChar
                if constexpr (Base::perms_type::canNotify && !Base::perms_type::canRead) {
                    // Direct notify without storing value
                    if constexpr (std::is_same_v<ValueT, std::string>) {
                        p->notify(reinterpret_cast<const uint8_t*>(newValue.data()), newValue.size());
                    } else if constexpr (std::is_same_v<ValueT, std::vector<uint8_t>>) {
                        p->notify(newValue.data(), newValue.size());
                    } else {
                        std::array<uint8_t, sizeof(ValueT)> buf{};
                        std::memcpy(buf.data(), &newValue, sizeof(ValueT));
                        p->notify(buf.data(), buf.size());
                    }
                } else {
                    // Characteristic is readable: must store value
                    if constexpr (std::is_same_v<ValueT, std::string>) {
                        p->setValue(newValue);
                    } else if constexpr (std::is_same_v<ValueT, std::vector<uint8_t>>) {
                        p->setValue(newValue.data(), newValue.size());
                    } else {
                        std::array<uint8_t, sizeof(ValueT)> buf{};
                        std::memcpy(buf.data(), &newValue, sizeof(ValueT));
                        p->setValue(buf.data(), buf.size());
                    }

                    if constexpr (Base::perms_type::canNotify) {
                        p->notify();
                    }
                }
            }

            if constexpr (use_read_notify_optimization) {
                read_notify_optimization.notified_value_valid.store(true, std::memory_order_release);
            }
        }

        static void setValue_unsafe(const uint8_t* data, size_t size) {
            if (auto* p = pChar) {  // Access outer scope pChar
                if constexpr (Base::perms_type::canNotify && !Base::perms_type::canRead) {
                    p->notify(data, size);
                } else {
                    p->setValue(data, size);
                    if constexpr (Base::perms_type::canNotify) {
                        p->notify();
                    }
                }
            }

            if constexpr (use_read_notify_optimization) {
                read_notify_optimization.notified_value_valid.store(true, std::memory_order_release);
            }
        }

    public:
        static void setValue(const typename Base::value_type& newValue) {
            if constexpr (Base::perms_type::canNotify && !Base::perms_type::canRead) {
                setValue_unsafe(newValue);
            } else {
                guard_t guard;
                setValue_unsafe(newValue);
            }
        }

        static void setValue(const uint8_t* data, size_t size) {
            if constexpr (Base::perms_type::canNotify && !Base::perms_type::canRead) {
                setValue_unsafe(data, size);
            } else {
                guard_t guard;
                setValue_unsafe(data, size);
            }
        }

        void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
            if constexpr (Base::ReadHandler != nullptr) {
                if constexpr (use_read_notify_optimization) {
                    if (read_notify_optimization.notified_value_valid.load(std::memory_order_acquire)) {
                        return;
                    }
                }

                guard_t guard;
                typename Base::value_type tmp;
                Base::ReadHandler(tmp);
                setValue_unsafe(tmp);
            }
        }

        void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
            if constexpr (Base::WriteHandler != nullptr) {
                if constexpr (std::is_same_v<typename Base::value_type, std::string> ||
                              std::is_same_v<typename Base::value_type, std::vector<uint8_t>>) {
                    Base::WriteHandler(pChar->getValue());
                } else {
                    const auto& data = pChar->getValue();
                    assert(data.size() >= sizeof(typename Base::value_type) &&
                           "BLE write data size mismatch");
                    typename Base::value_type val;
                    std::memcpy(&val, data.data(), sizeof(typename Base::value_type));
                    Base::WriteHandler(val);
                }
            }
        }

        void onStatus([[maybe_unused]] NimBLECharacteristic* pChar, [[maybe_unused]] int code) override {
            if constexpr (Base::StatusHandler != nullptr) {
                Base::StatusHandler(code);
            }
        }

        void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) override {
            if constexpr (use_read_notify_optimization) {
                if (subValue == 0) {
                    const int8_t prev = read_notify_optimization.subscriber_count.fetch_sub(1, std::memory_order_acq_rel);
                    assert(prev > 0 && "BUG: subscriber_count went negative");
                    if (prev == 1) {
                        read_notify_optimization.notified_value_valid.store(false, std::memory_order_release);
                    }
                } else {
                    read_notify_optimization.subscriber_count.fetch_add(1, std::memory_order_acq_rel);
                }
            }

            if constexpr (Base::SubscribeHandler != nullptr) {
                Base::SubscribeHandler(subValue);
            }
        }
    };

    // ---------------------- setValue Operations ----------------------

    /**
     * @brief Set characteristic value and notify subscribers (type-safe)
     * @tparam LockPolicy Lock implementation (FreeRTOSLock for multi-core, NoLock for single-core)
     */
    template<template<typename> class LockPolicy>
    static void setValue(const T& newValue) {
        Shim<LockPolicy>::setValue(newValue);
    }

    /**
     * @brief Set characteristic value from raw buffer
     * @note Used for array/struct types
     */
    template<template<typename> class LockPolicy>
    static void setValue(const uint8_t* data, size_t size) {
        Shim<LockPolicy>::setValue(data, size);
    }

    // ---------------------- Backend Registration ----------------------

    /**
     * @brief Register NimBLE characteristic handle and callbacks (encapsulated backend operation)
     * @tparam LockPolicy Lock implementation to use for callback shim
     * @param pC NimBLE characteristic pointer
     */
    template<template<typename> class LockPolicy>
    static void register_callbacks(NimBLECharacteristic* pC) {
        // Store characteristic handle
        pChar = pC;

        // Register callback shim if any callbacks are defined
        if constexpr (Base::ReadHandler != nullptr ||
                      Base::WriteHandler != nullptr ||
                      Base::StatusHandler != nullptr ||
                      Base::SubscribeHandler != nullptr) {
            static Shim<LockPolicy> shim;
            pC->setCallbacks(&shim);
        }
    }

    /**
     * @brief Register complete characteristic to NimBLE service (backend encapsulation)
     * @tparam LockPolicy Lock implementation to use
     * @param svc NimBLE service to register characteristic to
     * @return NimBLE characteristic pointer
     */
    template<template<typename> class LockPolicy>
    static NimBLECharacteristic* register_characteristic(NimBLEService* svc) {
        using PermsType = typename Base::perms_type;

        // Build property flags with security requirements
        uint16_t properties = 0;

        // Read permissions
        if constexpr (PermsType::canRead) {
            if constexpr (PermsType::requireAuthorization) {
                properties |= NIMBLE_PROPERTY::READ_AUTHOR;
            } else if constexpr (PermsType::requireAuthentication) {
                properties |= NIMBLE_PROPERTY::READ_AUTHEN;
            } else if constexpr (PermsType::requireEncryption) {
                properties |= NIMBLE_PROPERTY::READ_ENC;
            } else {
                properties |= NIMBLE_PROPERTY::READ;
            }
        }

        // Write permissions
        if constexpr (PermsType::canWrite) {
            if constexpr (PermsType::requireAuthorization) {
                properties |= NIMBLE_PROPERTY::WRITE_AUTHOR;
            } else if constexpr (PermsType::requireAuthentication) {
                properties |= NIMBLE_PROPERTY::WRITE_AUTHEN;
            } else if constexpr (PermsType::requireEncryption) {
                properties |= NIMBLE_PROPERTY::WRITE_ENC;
            } else {
                properties |= NIMBLE_PROPERTY::WRITE;
            }
        }

        // Write-no-response permissions
        if constexpr (PermsType::canWriteNoResponse) {
            properties |= NIMBLE_PROPERTY::WRITE_NR;
        }

        // Notify permission
        if constexpr (PermsType::canNotify) {
            properties |= NIMBLE_PROPERTY::NOTIFY;
        }

        // Indicate permission
        if constexpr (PermsType::canIndicate) {
            properties |= NIMBLE_PROPERTY::INDICATE;
        }

        // Create NimBLE characteristic
        NimBLECharacteristic* pC = svc->createCharacteristic(
            blex_nimble::make_uuid<Base::uuid>(),
            properties
        );

        // Set value for const characteristics
        if constexpr (Base::is_const_characteristic) {
            if constexpr (std::is_same_v<typename Base::value_type, std::string> ||
                          std::is_same_v<typename Base::value_type, const char*>)
                pC->setValue(Base::value);
            else
                pC->setValue(reinterpret_cast<const uint8_t*>(&Base::value), sizeof(typename Base::value_type));
        }

        // Register all descriptors inline (fold expression)
        register_all_descriptors(pC, typename Base::descriptors_pack{});

        // Register callbacks for non-const characteristics
        if constexpr (!Base::is_const_characteristic) {
            register_callbacks<LockPolicy>(pC);
        }

        return pC;
    }

private:
    /**
     * @brief Helper to register all descriptors (unwraps final types to Base for backend lookup)
     */
    template<typename... Descriptors>
    static void register_all_descriptors([[maybe_unused]] NimBLECharacteristic* pC, blex_core::DescriptorsPack<Descriptors...>) {
        if constexpr (sizeof...(Descriptors) > 0) {
            (DescriptorBackend<typename Descriptors::Base>::register_to_char(pC), ...);
        }
    }
};

#endif // BLEX_NIMBLE_AVAILABLE

// ---------------------- DescriptorBackend Trait Specializations (NimBLE) ----------------------

#ifdef BLEX_NIMBLE_AVAILABLE

/**
 * @brief NimBLE backend for generic DescriptorBase (handles both dynamic and const descriptors)
 */
template<typename T, auto UUID, typename Perms, size_t MaxSize>
struct DescriptorBackend<DescriptorBase<T, UUID, Perms, MaxSize>> {
    using Base = DescriptorBase<T, UUID, Perms, MaxSize>;

    static NimBLEDescriptor* register_to_char(NimBLECharacteristic* pChar) {
        NimBLEDescriptor* desc = pChar->createDescriptor(
            blex_nimble::make_uuid<UUID>(),
            (Perms::canRead ? NIMBLE_PROPERTY::READ : 0) | (Perms::canWrite ? NIMBLE_PROPERTY::WRITE : 0),
            MaxSize);
        return desc;
    }
};

/**
 * @brief NimBLE backend for PresentationFormatDescriptor
 */
template<uint8_t Format, int8_t Exponent, uint16_t Unit, uint8_t Namespace, uint16_t Description>
struct DescriptorBackend<PresentationFormatDescriptorBase<Format, Exponent, Unit, Namespace, Description>> {
    using Base = PresentationFormatDescriptorBase<Format, Exponent, Unit, Namespace, Description>;

    static NimBLEDescriptor* register_to_char(NimBLECharacteristic* pChar) {
        auto* desc = pChar->create2904();
        if (desc) {
            desc->setFormat(Format);
            desc->setExponent(Exponent);
            desc->setUnit(Unit);
            desc->setNamespace(Namespace);
            desc->setDescription(Description);
        }
        return desc;
    }
};

/**
 * @brief NimBLE backend for AggregateFormatDescriptor
 */
template<typename... PresentationFormatDescriptors>
struct DescriptorBackend<AggregateFormatDescriptorBase<PresentationFormatDescriptors...>> {
    using Base = AggregateFormatDescriptorBase<PresentationFormatDescriptors...>;

    template<typename PresentationDesc>
    static void add_presentation_descriptor(NimBLECharacteristic* pChar, NimBLE2905* p2905) {
        // Use the backend for the presentation descriptor
        auto* desc = DescriptorBackend<PresentationDesc>::register_to_char(pChar);
        if (desc) {
            p2905->add2904Descriptor(static_cast<NimBLE2904*>(desc));
        }
    }

    static NimBLEDescriptor* register_to_char(NimBLECharacteristic* pChar) {
        NimBLE2905* p2905 = pChar->create2905();
        (add_presentation_descriptor<PresentationFormatDescriptors>(pChar, p2905), ...);
        return p2905;
    }
};

// ---------------------- ServerBackend Trait Specialization (NimBLE) ----------------------

/**
 * @brief NimBLE backend implementation for Server
 * @details Provides runtime NimBLE server operations: initialization, advertising, connections
 * Inherits compile-time config from ServerBase.
 */
template<const char* DeviceName, const char* ShortName, typename Derived, typename... Args>
struct ServerBackend<ServerBase<DeviceName, ShortName, Derived, Args...>> {
    using Config = ServerBase<DeviceName, ShortName, Derived, Args...>;
    using ConnectionInfoType = NimBLEConnInfo;

    // Sentinel values for optional configuration
    static constexpr int8_t TX_POWER_UNSET = -127;
    static constexpr uint16_t APPEARANCE_UNSET = 0x0000;
    static constexpr uint8_t DEFAULT_BLE_FLAGS = 0x06;  // LE General Discoverable + BR/EDR Not Supported

    // NimBLE-specific runtime state
    inline static NimBLEServer* server = nullptr;
    inline static NimBLEAdvertising* adv = nullptr;

    // Runtime tuning state (static storage, no heap)
    inline static int8_t runtime_tx_power_ = TX_POWER_UNSET;
    inline static uint16_t runtime_adv_interval_min_ = 0;    // 0 = not set
    inline static uint16_t runtime_adv_interval_max_ = 0;    // 0 = not set

    // ---------------------- NimBLE Server Callbacks ----------------------

    struct Callbacks final : NimBLEServerCallbacks {
        template<auto Handler>
        static constexpr bool is_handler_provided = !std::is_same_v<std::remove_cv_t<decltype(Handler)>, std::nullptr_t>;

        void onConnect([[maybe_unused]] NimBLEServer* pServer, NimBLEConnInfo& nimble_conn) override {
            if constexpr (is_handler_provided<Config::ConnectHandler>) {
                Config::ConnectHandler(nimble_conn);
            } else {
                BLEX_LOG_INFO("🔗 Connected: %s\n", nimble_conn.getAddress().toString().c_str());
            }
        }

        void onDisconnect([[maybe_unused]] NimBLEServer* pServer, NimBLEConnInfo& nimble_conn, const int reason) override {
            if constexpr (is_handler_provided<Config::DisconnectHandler>) {
                Config::DisconnectHandler(nimble_conn, reason);
            } else {
                BLEX_LOG_INFO("Disconnected (reason=%d)\n", reason);
                NimBLEDevice::startAdvertising();
                BLEX_LOG_INFO("📡 Advertising restarted\n");
            }
        }

        void onMTUChange(const uint16_t MTU, NimBLEConnInfo& nimble_conn) override {
            if constexpr (is_handler_provided<Config::MTUChangeHandler>) {
                Config::MTUChangeHandler(nimble_conn);
            } else {
                BLEX_LOG_DEBUG("📏 MTU updated: %u bytes for %s\n", MTU, nimble_conn.getAddress().toString().c_str());

                server->updateConnParams(
                    nimble_conn.getConnHandle(),
                    Config::ConnConfig::conn_interval_min,
                    Config::ConnConfig::conn_interval_max,
                    Config::ConnConfig::conn_latency,
                    Config::ConnConfig::supervision_timeout
                );
                BLEX_LOG_DEBUG("📊 Requested connection parameters: interval=%u-%ums, latency=%u, timeout=%ums (%.1fs)\n",
                            Config::ConnConfig::conn_interval_min_ms, Config::ConnConfig::conn_interval_max_ms,
                            Config::ConnConfig::conn_latency,
                            Config::ConnConfig::supervision_timeout_ms, Config::ConnConfig::supervision_timeout_ms / 1000.0f);
            }
        }
    };

    // ---------------------- Initialization ----------------------

    [[nodiscard]]
    static bool init() {
        static std::atomic_flag init_called = ATOMIC_FLAG_INIT;
        if (init_called.test_and_set(std::memory_order_acq_rel)) {
            BLEX_LOG_WARN("[BLEX] init: already initialized, returning\n");
            return server != nullptr;
        }

        BLEX_LOG_DEBUG("🟢 Initializing BLE server...\n");
        BLEX_LOG_DEBUG("[BLEX] init: calling NimBLEDevice::init\n");
        NimBLEDevice::init(DeviceName);

        // Set BLE appearance in GAP service (if AdvConfig provided)
        if constexpr (!std::is_void_v<typename Config::AdvConfig>) {
            if constexpr (Config::AdvConfig::default_appearance != APPEARANCE_UNSET) {
                BLEX_LOG_DEBUG("[BLEX] init: setting GAP appearance to 0x%04X\n", Config::AdvConfig::default_appearance);
                ble_svc_gap_device_appearance_set(Config::AdvConfig::default_appearance);
            }
        }

        // Only set MTU if not using sentinel value
        if (Config::ConnConfig::mtu != 0) {
            BLEX_LOG_DEBUG("[BLEX] init: calling setMTU(%u)\n", Config::ConnConfig::mtu);
            NimBLEDevice::setMTU(Config::ConnConfig::mtu);
        }

        // Configure BLE security
        using SecConfig = typename Config::SecurityConfig;
        BLEX_LOG_DEBUG("[BLEX] init: configuring security (IO=%u, MITM=%d, Bonding=%d, SC=%d, Passkey=%u)\n",
                    SecConfig::io_capabilities, SecConfig::mitm_protection,
                    SecConfig::bonding, SecConfig::secure_connections, SecConfig::passkey);

        // Set IO capabilities
        NimBLEDevice::setSecurityIOCap(SecConfig::io_capabilities);

        // Set security authorization mode
        uint8_t auth_req = 0;
        if (SecConfig::bonding) auth_req |= BLE_SM_PAIR_AUTHREQ_BOND;
        if (SecConfig::mitm_protection) auth_req |= BLE_SM_PAIR_AUTHREQ_MITM;
        if (SecConfig::secure_connections) auth_req |= BLE_SM_PAIR_AUTHREQ_SC;
        NimBLEDevice::setSecurityAuth(auth_req);

        // Set static passkey if configured
        if constexpr (SecConfig::passkey != 0) {
            BLEX_LOG_DEBUG("[BLEX] init: setting static passkey\n");
            NimBLEDevice::setSecurityPasskey(SecConfig::passkey);
        }

        BLEX_LOG_DEBUG("[BLEX] init: creating server\n");
        server = NimBLEDevice::createServer();
        if (!server) {
            BLEX_LOG_ERROR("Failed to create BLE server\n");
            return false;
        }

        BLEX_LOG_DEBUG("[BLEX] init: setting callbacks\n");
        static Callbacks callbacks;
        server->setCallbacks(&callbacks);

        BLEX_LOG_DEBUG("[BLEX] init: getting advertising\n");
        adv = NimBLEDevice::getAdvertising();

        return true;
    }

    // ---------------------- Advertising Control ----------------------

    static void startAdvertising() {
        BLEX_LOG_DEBUG("[BLEX] init: starting advertising\n");
        NimBLEDevice::startAdvertising();
    }

    static void stopAdvertising() {
        NimBLEDevice::stopAdvertising();
    }

    // ---------------------- Connection Management ----------------------

    [[nodiscard]]
    static bool isConnected() {
        return server ? server->getConnectedCount() > 0 : false;
    }

    [[nodiscard]]
    static uint16_t getConnectedCount() {
        return server ? server->getConnectedCount() : 0;
    }

    static void disconnect(uint16_t conn_handle) {
        server->disconnect(conn_handle);
    }

    [[nodiscard]]
    static int8_t getRSSI(uint16_t conn_handle) {
        if (!server) return 0;
        auto* conn_info = server->getPeerInfo(conn_handle);
        return conn_info ? conn_info->getRSSI() : 0;
    }

    // ---------------------- Runtime Configuration ----------------------

    /**
     * @brief Set TX power at runtime (validates against hardware limits)
     * @param dbm TX power in dBm (ESP32-S3 range: -12 to +9)
     * @return true if valid and applied, false if out of range
     */
    [[nodiscard]]
    static bool setTxPower(int8_t dbm) {
        // ESP32-S3 hardware limits
        constexpr int8_t min_tx = -12;
        constexpr int8_t max_tx = 9;

        if (dbm < min_tx || dbm > max_tx) {
            BLEX_LOG_ERROR("TX power %d dBm out of range [%d, %d]\n", dbm, min_tx, max_tx);
            return false;
        }
        runtime_tx_power_ = dbm;
        return true;
    }

    /**
     * @brief Set advertising interval at runtime (validates against BLE spec)
     * @param min_ms Minimum interval in milliseconds (BLE range: 20-10240)
     * @param max_ms Maximum interval in milliseconds (BLE range: 20-10240)
     * @return true if valid and applied, false if out of range
     */
    [[nodiscard]]
    static bool setAdvInterval(uint16_t min_ms, uint16_t max_ms) {
        // BLE spec limits
        constexpr uint16_t min_adv = 20;
        constexpr uint16_t max_adv = 10240;

        if (min_ms < min_adv || min_ms > max_adv ||
            max_ms < min_adv || max_ms > max_adv ||
            min_ms > max_ms) {
            BLEX_LOG_ERROR("Advertising interval out of range [%u, %u] or min > max\n", min_adv, max_adv);
            return false;
        }
        runtime_adv_interval_min_ = min_ms;
        runtime_adv_interval_max_ = max_ms;
        BLEX_LOG_DEBUG("Advertising interval set to [%u, %u] ms (call updateAdvertising to apply)\n",
                    min_ms, max_ms);
        return true;
    }

    /**
     * @brief Configure BLE advertising with two-layer priority system
     * @details Applies advertising configuration using priority:
     *          1. Runtime state (set via setTxPower/setAdvInterval)
     *          2. Compile-time defaults (from AdvertisingConfig template)
     *          3. NimBLE stack defaults (if neither provided)
     *
     * @tparam PassiveServices Tuple of services advertised in main packet
     * @tparam ActiveServices Tuple of services advertised in scan response
     * @param device_name Full device name (scan response)
     * @param short_name Short name (advertising packet)
     */
    template<typename PassiveServices, typename ActiveServices>
    static void configureAdvertising(const char* device_name, const char* short_name) {
        using AdvConfig = typename Config::AdvConfig;

        if (!adv) return;

        // Enable scan response for extended data
        adv->enableScanResponse(true);

        // Configure advertisement data (passive services + short name + manufacturer data)
        NimBLEAdvertisementData adv_data;

        if constexpr (!std::is_void_v<AdvConfig>) {
            adv_data.setFlags(AdvConfig::default_flags);
        } else {
            adv_data.setFlags(DEFAULT_BLE_FLAGS);
        }

        adv_data.setName(short_name, false);
        blex_nimble::add_service_uuids_impl(adv_data, PassiveServices{});

        // Add manufacturer data if configured (compile-time only)
        if constexpr (!std::is_void_v<AdvConfig>) {
            if constexpr (AdvConfig::manufacturer_data != nullptr && AdvConfig::manufacturer_data_len > 0) {
                adv_data.setManufacturerData(
                    std::string(reinterpret_cast<const char*>(AdvConfig::manufacturer_data),
                               AdvConfig::manufacturer_data_len)
                );
            }
        }

        adv->setAdvertisementData(adv_data);

        // Configure scan response data (active services + full name)
        NimBLEAdvertisementData scan_resp;
        scan_resp.setName(device_name, true);
        blex_nimble::add_service_uuids_impl(scan_resp, ActiveServices{});
        adv->setScanResponseData(scan_resp);

        // Apply TX power with two-layer priority
        int8_t tx_power;
        if (runtime_tx_power_ != TX_POWER_UNSET) {
            // Layer 1: Runtime value (set via setTxPower)
            tx_power = runtime_tx_power_;
        } else if constexpr (!std::is_void_v<AdvConfig>) {
            // Layer 2: Compile-time default
            tx_power = AdvConfig::default_tx_power;
        } else {
            // Layer 3: Sentinel (use NimBLE default)
            tx_power = TX_POWER_UNSET;
        }

        // Only set TX power if not using sentinel value
        if (tx_power != TX_POWER_UNSET) {
            NimBLEDevice::setPower(tx_power);
        }

        // Apply advertising intervals with two-layer priority
        uint16_t interval_min, interval_max;
        if (runtime_adv_interval_min_ != 0 && runtime_adv_interval_max_ != 0) {
            // Layer 1: Runtime values (set via setAdvInterval)
            interval_min = runtime_adv_interval_min_;
            interval_max = runtime_adv_interval_max_;
        } else if constexpr (!std::is_void_v<AdvConfig>) {
            // Layer 2: Compile-time defaults
            interval_min = AdvConfig::default_adv_interval_min;
            interval_max = AdvConfig::default_adv_interval_max;
        } else {
            // Layer 3: Sentinel (use NimBLE defaults)
            interval_min = 0;
            interval_max = 0;
        }

        // Only set intervals if not using sentinel values (0 = use NimBLE defaults)
        if (interval_min != 0 && interval_max != 0) {
            // NimBLE uses 0.625ms units, so convert milliseconds
            adv->setMinInterval(interval_min * 16 / 10);
            adv->setMaxInterval(interval_max * 16 / 10);
        }

        // Set BLE appearance in advertising packet (compile-time only)
        if constexpr (!std::is_void_v<AdvConfig>) {
            if constexpr (AdvConfig::default_appearance != APPEARANCE_UNSET) {
                adv->setAppearance(AdvConfig::default_appearance);
            }
        }
    }

    /**
     * @brief Start a registered BLE service (backend-specific implementation)
     * @tparam Service The service type to start (must be already registered)
     *
     * @details Looks up the service by UUID and starts it if found.
     *          This encapsulates the NimBLE-specific UUID conversion and
     *          service lookup logic within the backend.
     */
    template<typename Service>
    static void startService() {
        if (auto* s = server->getServiceByUUID(blex_nimble::make_uuid<Service::uuid>())) {
            s->start();
        }
    }
};

#endif // BLEX_NIMBLE_AVAILABLE

#endif // BLEX_NIMBLE_HPP_