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
        #if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        BLEX_LOG_TRACE("Registering service %s\n", blex_nimble::make_uuid<UUID>().toString().c_str());
        #endif

        // Create NimBLE service
        pService = server->createService(blex_nimble::make_uuid<UUID>());

        // Register all characteristics inline (blex<> will be complete when this is instantiated)
        register_all_chars<LockPolicy>(typename Base::chars_pack{});

        #if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        BLEX_LOG_DEBUG("Registered service %s\n", blex_nimble::make_uuid<UUID>().toString().c_str());
        #endif
    }

    /**
     * @brief Start the service (makes it visible in GATT table)
     * @return true if successful, false if service not registered
     */
    static bool start() {
        if (!pService) {
            BLEX_LOG_ERROR("Service not registered (pService is null)\n");
            return false;
        }

        if (!pService->isStarted()) {
            pService->start();
        }
        return true;
    }

    /**
     * @brief Stop the service (remove from server but keep object for re-adding)
     * @return true if successful, false if service not found
     */
    static bool stop() {
        if (!pService) {
            BLEX_LOG_ERROR("Service not registered (pService is null)\n");
            return false;
        }
        NimBLEServer* server = pService->getServer();
        if (!server) {
            BLEX_LOG_ERROR("Server not initialized for service\n");
            return false;
        }

        // Remove service but keep object (deleteSvc=false) so it can be re-added later
        server->removeService(pService, false);
        return true;
    }

    /**
     * @brief Check if service is currently started
     * @return true if started, false otherwise
     */
    [[nodiscard]]
    static bool isStarted() {
        return pService ? pService->isStarted() : false;
    }

private:
    /**
     * @brief Helper to register all characteristics (directly to backends)
     */
    template<template<typename> class LockPolicy, typename... CharTypes>
    static void register_all_chars(blex_core::CharsPack<CharTypes...>) {
        // Register each characteristic and log the actual NimBLE configuration
        #if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        ([]{
            using CharPermsType = typename CharTypes::Base::perms_type;
            BLEX_LOG_TRACE("  Registering char %s (security: read=%d, write=%d)\n",
                blex_nimble::make_uuid<CharTypes::Base::uuid>().toString().c_str(),
                CharPermsType::read_security,
                CharPermsType::write_security);
            auto* pChar = CharTypes::Backend::template register_characteristic<LockPolicy>(pService);
            BLEX_LOG_DEBUG("  Registered char UUID=%s props=0x%04X descs=%zu\n",
                pChar->getUUID().toString().c_str(),
                pChar->getProperties(),
                std::tuple_size_v<typename CharTypes::Base::descriptors_pack>);
        }(), ...);
        #else
        (CharTypes::Backend::template register_characteristic<LockPolicy>(pService), ...);
        #endif
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
        // NimBLE requires BOTH the base flag (for property advertisement) AND the security flag (for enforcement)
        if constexpr (PermsType::canRead) {
            properties |= NIMBLE_PROPERTY::READ;  // Always set a base flag for property advertisement

            if constexpr (PermsType::read_security >= 4) {
                properties |= NIMBLE_PROPERTY::READ_AUTHOR ;
                BLEX_LOG_DEBUG("    char: READ | READ_AUTHOR\n");
            } else if constexpr (PermsType::read_security >= 3) {
                properties |= NIMBLE_PROPERTY::READ_AUTHEN;
                BLEX_LOG_DEBUG("    char: READ | READ_AUTHEN\n");
            } else if constexpr (PermsType::read_security >= 2) {
                properties |= NIMBLE_PROPERTY::READ_ENC;
                BLEX_LOG_DEBUG("    char: READ | READ_ENC\n");
            } else {
                BLEX_LOG_DEBUG("    char: READ\n");
            }
        }

        // Write permissions
        // NimBLE requires BOTH the base flag (for property advertisement) AND the security flag (for enforcement)
        if constexpr (PermsType::canWrite) {
            properties |= NIMBLE_PROPERTY::WRITE;  // Always set a base flag for property advertisement

            // Mapping for macOS / BLE spec
            if constexpr (PermsType::write_security >= 4) {
                properties |= NIMBLE_PROPERTY::WRITE_AUTHOR;;
                BLEX_LOG_DEBUG("    char: WRITE | WRITE_AUTHOR\n");
            } else if constexpr (PermsType::write_security >= 3) {
                properties |= NIMBLE_PROPERTY::WRITE_AUTHEN;
                BLEX_LOG_DEBUG("    char: WRITE | WRITE_AUTHEN\n");
            } else if constexpr (PermsType::write_security >= 2) {
                properties |= NIMBLE_PROPERTY::WRITE_ENC;
                BLEX_LOG_DEBUG("    char: WRITE | WRITE_ENC\n");
            } else {
                BLEX_LOG_DEBUG("    char: WRITE\n");
            }
        }

        // Write-no-response permissions
        if constexpr (PermsType::canWriteNoResponse) {
            properties |= NIMBLE_PROPERTY::WRITE_NR;
            BLEX_LOG_DEBUG("    char: WRITE_NR\n");
        }

        // Notify permission
        if constexpr (PermsType::canNotify) {
            properties |= NIMBLE_PROPERTY::NOTIFY;
            BLEX_LOG_DEBUG("    char: NOTIFY\n");
        }

        // Indicate permission
        if constexpr (PermsType::canIndicate) {
            properties |= NIMBLE_PROPERTY::INDICATE;
            BLEX_LOG_DEBUG("    char: INDICATE\n");
        }

        // Broadcast permission
        if constexpr (PermsType::canBroadcast) {
            properties |= NIMBLE_PROPERTY::BROADCAST;
            BLEX_LOG_DEBUG("    char: BROADCAST\n");
        }

        // Create NimBLE characteristic
        NimBLECharacteristic* pC = svc->createCharacteristic(
            blex_nimble::make_uuid<Base::uuid>(),
            properties
        );

        /// Set value for const characteristics
        if constexpr (Derived::is_const_characteristic) {
            if constexpr (std::is_same_v<typename Base::value_type, std::string> ||
                          std::is_same_v<typename Base::value_type, const char*>) {
                pC->setValue(std::string(Derived::value));
            } else {
                pC->setValue(reinterpret_cast<const uint8_t*>(&Derived::value), sizeof(typename Base::value_type));
            }
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
     * @brief Helper to register all descriptors (passes full descriptor types for proper backend specialization)
     */
    template<typename... Descriptors>
    static void register_all_descriptors([[maybe_unused]] NimBLECharacteristic* pC, blex_core::DescriptorsPack<Descriptors...>) {
        if constexpr (sizeof...(Descriptors) > 0) {
            (DescriptorBackend<Descriptors>::register_to_char(pC), ...);
        }
    }
};

#endif // BLEX_NIMBLE_AVAILABLE

// ---------------------- DescriptorBackend Trait Specializations (NimBLE) ----------------------

#ifdef BLEX_NIMBLE_AVAILABLE

/**
 * @brief NimBLE backend for descriptors (handles both const and dynamic descriptors)
 * @details Checks is_const_descriptor flag and marker traits for special descriptor types
 */
template<typename Desc>
struct DescriptorBackend {
private:
    // Check if descriptor has is_const_descriptor flag (SFINAE fallback for types without the flag)
    template<typename D>
    static constexpr auto check_const_flag(int) -> decltype(D::is_const_descriptor) {
        return D::is_const_descriptor;
    }

    template<typename D>
    static constexpr bool check_const_flag(...) {
        return false;
    }

    // Check if descriptor has is_presentation_format_descriptor marker (SFINAE fallback)
    template<typename D>
    static constexpr auto check_presentation_format(int) -> decltype(typename D::is_presentation_format_descriptor{}, std::true_type{}) {
        return std::true_type{};
    }

    template<typename D>
    static constexpr std::false_type check_presentation_format(...) {
        return std::false_type{};
    }

    // Check if descriptor has is_aggregate_format_descriptor marker (SFINAE fallback)
    template<typename D>
    static constexpr auto check_aggregate_format(int) -> decltype(typename D::is_aggregate_format_descriptor{}, std::true_type{}) {
        return std::true_type{};
    }

    template<typename D>
    static constexpr std::false_type check_aggregate_format(...) {
        return std::false_type{};
    }

    static constexpr bool is_const_descriptor = check_const_flag<Desc>(0);
    static constexpr bool is_presentation_format = decltype(check_presentation_format<Desc>(0))::value;
    static constexpr bool is_aggregate_format = decltype(check_aggregate_format<Desc>(0))::value;

public:
    static NimBLEDescriptor* register_to_char(NimBLECharacteristic* pChar) {
        // Delegate to Base specialization for presentation format descriptors
        if constexpr (is_presentation_format) {
            using Base = typename Desc::Base;
            return DescriptorBackend<Base>::register_to_char(pChar);
        }
        // Delegate to Base specialization for aggregate format descriptors
        else if constexpr (is_aggregate_format) {
            using Base = typename Desc::Base;
            return DescriptorBackend<Base>::register_to_char(pChar);
        }
        // Generic descriptor handling
        else {
            using Base = typename Desc::Base;

            NimBLEDescriptor* desc = pChar->createDescriptor(
                blex_nimble::make_uuid<Base::uuid>(),
                (Base::perms_type::canRead ? NIMBLE_PROPERTY::READ : 0) |
                (Base::perms_type::canWrite ? NIMBLE_PROPERTY::WRITE : 0),
                Base::max_size);

            // Set value for const descriptors (checked via is_const_descriptor flag)
            if constexpr (is_const_descriptor) {
                if (desc) {
                    using T = typename Base::value_type;
                    if constexpr (std::is_same_v<T, const char*>) {
                        desc->setValue(std::string(Desc::value));
                    } else if constexpr (std::is_same_v<T, std::string>) {
                        desc->setValue(Desc::value);
                    } else {
                        desc->setValue(reinterpret_cast<const uint8_t*>(&Desc::value), sizeof(T));
                    }
                }
            }
            return desc;
        }
    }
};

/**
 * @brief NimBLE backend for PresentationFormatDescriptorBase
 * @details Handles both Base and derived PresentationFormatDescriptor types via trait detection
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
template<const char* ShortName, typename Derived, typename... Args>
struct ServerBackend<ServerBase<ShortName, Derived, Args...>> {
    using Config = ServerBase<ShortName, Derived, Args...>;
    using connection_handle_t = uint16_t;
    using ConnectionInfoType = NimBLEConnInfo;

    // Sentinel values for connection handles and optional configuration
    static constexpr connection_handle_t InvalidConnHandle = 0xFFFF;  // BLE_HS_CONN_HANDLE_NONE
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
                BLEX_LOG_INFO("Connected: %s\n", nimble_conn.getAddress().toString().c_str());
            }
        }

        void onDisconnect([[maybe_unused]] NimBLEServer* pServer, NimBLEConnInfo& nimble_conn, const int reason) override {
            if constexpr (is_handler_provided<Config::DisconnectHandler>) {
                Config::DisconnectHandler(nimble_conn, reason);
            } else {
                BLEX_LOG_INFO("Disconnected (reason=%d)\n", reason);
                NimBLEDevice::startAdvertising();
                BLEX_LOG_INFO("Advertising restarted\n");
            }
        }

        void onMTUChange(const uint16_t MTU, NimBLEConnInfo& nimble_conn) override {
            if constexpr (is_handler_provided<Config::MTUChangeHandler>) {
                Config::MTUChangeHandler(nimble_conn);
            } else {
                BLEX_LOG_INFO("MTU updated: %u bytes for %s\n", MTU, nimble_conn.getAddress().toString().c_str());

                // Update connection parameters if ConnectionConfig is provided
                if constexpr (!std::is_void_v<typename Config::ConnConfig>) {
                    server->updateConnParams(
                        nimble_conn.getConnHandle(),
                        Config::ConnConfig::conn_interval_min,
                        Config::ConnConfig::conn_interval_max,
                        Config::ConnConfig::conn_latency,
                        Config::ConnConfig::supervision_timeout
                    );
                    BLEX_LOG_INFO("Requested connection parameters: interval=%u-%ums, latency=%u, timeout=%ums (%.1fs)\n",
                                Config::ConnConfig::conn_interval_min_ms, Config::ConnConfig::conn_interval_max_ms,
                                Config::ConnConfig::conn_latency,
                                Config::ConnConfig::supervision_timeout_ms, Config::ConnConfig::supervision_timeout_ms / 1000.0f);
                }
            }
        }
    };

    // ---------------------- Initialization ----------------------

    /**
     * @brief Ensure server is initialized (auto-initializes if needed)
     * @return true if initialized (or successfully auto-initialized), false otherwise
     */
    [[nodiscard]]
    static bool ensure_initialized() {
        if (!server) {
            return init();
        }
        return true;
    }

    [[nodiscard]]
    static bool init() {
        BLEX_LOG_DEBUG("init: initializing NimBLE-Arduino backend...\n");

        static std::atomic_flag init_called = ATOMIC_FLAG_INIT;
        if (init_called.test_and_set(std::memory_order_acq_rel)) {
            BLEX_LOG_WARN("NimBLE-Arduino backend already initialized, nothing to do\n");
            return server != nullptr;
        }

        BLEX_LOG_TRACE("init: calling NimBLEDevice::init\n");
        NimBLEDevice::init(Config::device_name);

        // Set BLE appearance in GAP service (if AdvConfig provided)
        if constexpr (!std::is_void_v<typename Config::AdvConfig>) {
            if constexpr (Config::AdvConfig::default_appearance != APPEARANCE_UNSET) {
                BLEX_LOG_TRACE("init: setting GAP appearance to 0x%04X\n", Config::AdvConfig::default_appearance);
                ble_svc_gap_device_appearance_set(Config::AdvConfig::default_appearance);
            }
        }

        // Configure connection parameters if provided
        if constexpr (!std::is_void_v<typename Config::ConnConfig>) {
            // Only set MTU if specified
            if constexpr (Config::ConnConfig::mtu != 0) {
                BLEX_LOG_TRACE("init: calling setMTU\n");
                NimBLEDevice::setMTU(Config::ConnConfig::mtu);
                BLEX_LOG_DEBUG("init: MTU is set to %u\n", Config::ConnConfig::mtu);
            }
        }

        // Configure BLE security if user provided SecurityConfig
        if constexpr (!std::is_void_v<typename Config::SecurityConfig>) {
            BLEX_LOG_TRACE("init: configuring security...\n");

            // Set IO capabilities
            NimBLEDevice::setSecurityIOCap(Config::SecurityConfig::io_capabilities);

            // Set security authorization mode
            uint8_t auth_req = 0;
            if (Config::SecurityConfig::bonding) auth_req |= BLE_SM_PAIR_AUTHREQ_BOND;
            if (Config::SecurityConfig::mitm_protection) auth_req |= BLE_SM_PAIR_AUTHREQ_MITM;
            if (Config::SecurityConfig::secure_connections) auth_req |= BLE_SM_PAIR_AUTHREQ_SC;
            NimBLEDevice::setSecurityAuth(auth_req);

            // Set a static passkey if configured
            if constexpr (Config::SecurityConfig::passkey != 0) {
                BLEX_LOG_TRACE("init: setting static passkey\n");
                NimBLEDevice::setSecurityPasskey(Config::SecurityConfig::passkey);
            }
            BLEX_LOG_DEBUG("init: security configured: IO=%u, MITM=%d, Bonding=%d, SC=%d, Passkey=%u)\n",
                Config::SecurityConfig::io_capabilities, Config::SecurityConfig::mitm_protection,
                Config::SecurityConfig::bonding, Config::SecurityConfig::secure_connections, Config::SecurityConfig::passkey);

        }

        BLEX_LOG_TRACE("init: creating server\n");
        server = NimBLEDevice::createServer();
        if (!server) {
            BLEX_LOG_ERROR("init: NimBLE-Arduino backend failed to create BLE server\n");
            return false;
        }

        BLEX_LOG_TRACE("init: setting callbacks\n");
        static Callbacks callbacks;
        server->setCallbacks(&callbacks);

        BLEX_LOG_TRACE("init: getting advertising\n");
        adv = NimBLEDevice::getAdvertising();

        BLEX_LOG_INFO("init: NimBLE-Arduino backend successfully initialized\n");
        return true;
    }

    // ---------------------- Advertising Control ----------------------

    static void startAdvertising() {
        BLEX_LOG_DEBUG("Staring advertisement...\n");
        NimBLEDevice::startAdvertising();
        BLEX_LOG_INFO("Advertising started\n");
    }

    static void stopAdvertising() {
        BLEX_LOG_DEBUG("Stopping advertisement...\n");
        NimBLEDevice::stopAdvertising();
        BLEX_LOG_INFO("Advertising stopped\n");
    }

    [[nodiscard]]
    static bool isAdvertising() {
        return adv ? adv->isAdvertising() : false;
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

    /**
     * @brief Disconnect peer connection(s)
     * @param conn_handle Connection handle to disconnect (default: InvalidConnHandle = disconnect all)
     * @return true if successful, false if server not initialized, handle invalid, or disconnect failed
     */
    [[nodiscard]]
    static bool disconnect(connection_handle_t conn_handle = InvalidConnHandle) {
        if (!server) return false;

        if (conn_handle == InvalidConnHandle) {
            // Disconnect all peers - cache connection list
            std::vector<ConnectionInfoType> connections = getConnections();
            bool all_success = true;
            for (const ConnectionInfoType& conn : connections) {
                int rc = server->disconnect(conn.getConnHandle());
                if (rc != 0) {
                    BLEX_LOG_ERROR("Failed to disconnect handle %u: error %d\n", conn.getConnHandle(), rc);
                    all_success = false;
                }
            }
            return all_success;
        } else {
            // Disconnect specific peer - validate handle exists first
            std::vector<ConnectionInfoType> connections = getConnections();
            bool found = false;
            for (const ConnectionInfoType& conn : connections) {
                if (conn.getConnHandle() == conn_handle) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                BLEX_LOG_ERROR("Invalid connection handle: %u\n", conn_handle);
                return false;
            }

            int rc = server->disconnect(conn_handle);
            if (rc != 0) {
                BLEX_LOG_ERROR("Failed to disconnect handle %u: error %d\n", conn_handle, rc);
                return false;
            }

            return true;
        }
    }

    /**
     * @brief Get RSSI for a specific connection
     * @param conn_handle Connection handle
     * @return RSSI in dBm (0 if server not initialized)
     */
    [[nodiscard]]
    static int8_t getRSSI(uint16_t conn_handle) {
        if (!server) return 0;
        int8_t rssi = 0;
        ble_gap_conn_rssi(conn_handle, &rssi);
        return rssi;
    }

    /**
     * @brief Get the device's own BLE address
     * @return Pointer to static string containing address (cached on first call)
     */
    [[nodiscard]]
    static const char* getAddress() {
        static const std::string addr_str = NimBLEDevice::getAddress().toString();
        return addr_str.c_str();
    }

    /**
     * @brief Get connection info for a specific connection
     * @param handle Connection handle
     * @return NimBLEConnInfo object by value, or default-constructed if server not initialized
     * @note Returns by value since NimBLE API returns temporary objects
     */
    [[nodiscard]]
    static ConnectionInfoType getConnectionInfo(connection_handle_t handle) {
        static ConnectionInfoType default_info{};
        if (!server) return default_info;
        return server->getPeerInfo(handle);
    }

    /**
     * @brief Get all active connections
     * @return Vector of NimBLEConnInfo objects for all connected peers
     */
    [[nodiscard]]
    static std::vector<ConnectionInfoType> getConnections() {
        std::vector<ConnectionInfoType> result;
        if (!server) return result;

        for (auto handle : server->getPeerDevices()) {
            result.push_back(server->getPeerInfo(handle));
        }
        return result;
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
     * @brief Configure BLE advertising with a two-layer priority system
     * @details Applies advertising configuration using priority:
     *          1. Runtime state (set via setTxPower/setAdvInterval)
     *          2. Compile-time defaults (from AdvertisementConfig template)
     *          3. NimBLE stack defaults (if neither provided)
     *
     * Names are read from Config (ServerBase):
     *   - short_name: Short name for advertising packet
     *   - device_name: Long name if AdvConfig::long_name set, otherwise short_name
     *
     * @tparam PassiveServices Tuple of services advertised in the main packet
     * @tparam ActiveServices Tuple of services advertised in scan response
     */
    template<typename PassiveServices, typename ActiveServices>
    static void configureAdvertising() {
        using AdvConfig = typename Config::AdvConfig;

        if (!adv) return;

        constexpr const char* short_name = Config::short_name;
        constexpr const char* device_name = Config::device_name;

        BLEX_LOG_DEBUG("Advertising setup:\n");
        BLEX_LOG_DEBUG("  short_name = \"%s\" (addr: %p)\n", short_name, (void*)short_name);
        BLEX_LOG_DEBUG("  device_name = \"%s\" (addr: %p)\n", device_name, (void*)device_name);

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

        // Enable scan response if long name OR active services are configured
        constexpr bool has_active_services = !std::is_same_v<ActiveServices, std::tuple<>>;
        constexpr bool has_long_name = (device_name != short_name);

        BLEX_LOG_DEBUG("  has_active_services = %s\n", has_active_services ? "true" : "false");
        BLEX_LOG_DEBUG("  has_long_name = %s\n", has_long_name ? "true" : "false");

        if constexpr (has_long_name || has_active_services) {
            // Enable scan response for extended data
            adv->enableScanResponse(true);
            BLEX_LOG_DEBUG("  Scan response ENABLED\n");

            // Configure scan response data (active services + name)
            NimBLEAdvertisementData scan_resp;

            // Use device_name (which is long name if provided, otherwise short_name)
            if constexpr (has_long_name) {
                BLEX_LOG_DEBUG("  Setting scan response name to: \"%s\" (long name)\n", device_name);
                scan_resp.setName(device_name, true);
            } else {
                BLEX_LOG_DEBUG("  Setting scan response name to: \"%s\" (short name)\n", short_name);
                scan_resp.setName(short_name, false);
            }

            blex_nimble::add_service_uuids_impl(scan_resp, ActiveServices{});
            adv->setScanResponseData(scan_resp);
        } else {
            // No scan response data configured - disable scan response (passive only)
            adv->enableScanResponse(false);
        }

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