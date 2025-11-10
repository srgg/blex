/**
 * @file core.hpp
 * @brief Template metaprogramming engine - zero-cost compile-time BLE configuration
 *
 * @details
 * Core layer providing template metaprogramming primitives, type traits, and
 * compile-time validation for the BLEX framework. All constructs here are pure
 * C++20 with zero runtime cost - everything resolves at compile time.
 *
 * # Layer Responsibilities
 * - C++20 concepts for type validation
 * - CRTP base classes for characteristics, services, and servers
 * - Fluent builder configuration types (Permissions, AdvertisingConfig, etc.)
 * - Compile-time metaprogramming utilities (type filtering, extraction)
 * - Backend-agnostic metadata definitions
 *
 * @note Backend-agnostic: No NimBLE or platform dependencies
 * @see blex.hpp for API layer, nimble.hpp for backend implementation
 */

#ifndef BLEX_CORE_HPP_
#define BLEX_CORE_HPP_

#include <tuple>
#include <type_traits>
#include <cstring>
#include <cstdint>

// Forward declarations
namespace blex_standard {
    enum class BleAppearance : uint16_t;
}

// Forward declare blex template for friendship
template<template<typename> class LockPolicy>
struct blex;

namespace blex_core {

// ---------------------- C++20 Concepts ----------------------

// UUID type concept
template<typename T>
concept UuidType = requires {
    requires std::is_integral_v<std::remove_cv_t<std::remove_reference_t<T>>> ||
             std::is_same_v<std::remove_cv_t<std::remove_pointer_t<std::remove_cv_t<std::remove_reference_t<T>>>>, char> ||
             std::is_same_v<std::remove_cv_t<std::remove_extent_t<std::remove_cv_t<std::remove_reference_t<T>>>>, char>;
};

// Service wrapper concept
template<typename T>
concept ServiceWrapper = requires { typename T::service_type; };

// Advertising config concept
template<typename T>
concept AdvertisingConfigType = requires { typename T::is_blex_advertising_config_tag; };

// Connection config concept
template<typename T>
concept ConnectionConfigType = requires { typename T::is_blex_connection_config_tag; };

// Security config concept
template<typename T>
concept SecurityConfigType = requires { typename T::is_blex_security_config_tag; };

// Server callbacks config concept
template<typename T>
concept ServerCallbacksConfigType = requires { typename T::is_blex_server_callbacks_config_tag; };

// Characteristic callbacks config concept
template<typename T>
concept CharCallbacksConfigType = requires { typename T::is_blex_char_callbacks_config_tag; };

// Presentation format descriptor concept
template<typename T>
concept IsPresentationFormatDescriptor = requires { typename T::is_presentation_format_descriptor; };

// ---------------------- Type Traits and Metaprogramming ----------------------

// UUID type validation (compile-time evaluation)
template<typename T>
static constexpr void check_uuid_type() {
    static_assert(UuidType<T>, "UUID must be an integer (e.g., uint16_t) or a C string (char*/char[])");
}

// Compile-time string length helper
static constexpr size_t const_strlen(const char* str) {
    return *str ? 1 + const_strlen(str + 1) : 0;
}

// Value storage size calculation
template<typename T, T = T{}>
struct value_storage_size {
    static constexpr size_t value = sizeof(T);
};

template<const char* Val>
struct value_storage_size<const char*, Val> {
    static constexpr size_t value = const_strlen(Val) + 1;
};

// Helper variable template
template<typename T, T Val = T{}>
static constexpr size_t value_storage_size_v = value_storage_size<T, Val>::value;

// Unwrap service type (concept-based with SFINAE for member access)
template<typename T, typename = void>
struct unwrap_service_impl {
    using type = T;
};

template<typename T>
struct unwrap_service_impl<T, std::enable_if_t<ServiceWrapper<T>>> {
    using type = typename T::service_type;
};

template<typename T>
using unwrap_service = typename unwrap_service_impl<T>::type;

// Pack wrappers to hold types as template parameter packs (not tuples)
template<typename... /*Services*/>
struct ServicesPack {};

template<typename... /*Chars*/>
struct CharsPack {};

template<typename... /*Descs*/>
struct DescriptorsPack {};

// Service filtering (returns a tuple for advertising configuration)
template<template<typename> class Predicate, typename... Services>
struct filter_services;

template<template<typename> class Predicate>
struct filter_services<Predicate> {
    using type = std::tuple<>;
};

template<template<typename> class Predicate, typename First, typename... Rest>
struct filter_services<Predicate, First, Rest...> {
    using filtered_rest = typename filter_services<Predicate, Rest...>::type;
    using type = std::conditional_t<
        Predicate<First>::value,
        decltype(std::tuple_cat(std::tuple<First>{}, filtered_rest{})),
        filtered_rest
    >;
};

// Apply filter_services to ServicesPack
template<template<typename> class Predicate, typename SvcPack>
struct filter_services_pack;

template<template<typename> class Predicate, typename... Services>
struct filter_services_pack<Predicate, ServicesPack<Services...>> {
    using type = typename filter_services<Predicate, Services...>::type;
};

// Presentation Format descriptor detection (concept-based)
template<typename T>
static constexpr bool has_presentation_format_marker() {
    return IsPresentationFormatDescriptor<T>;
}

// Predicates for filtering (concept-based with SFINAE fallback for member access)
template<typename T, typename = void>
struct is_passive_adv_pred : std::false_type {};

template<typename T>
struct is_passive_adv_pred<T, std::enable_if_t<ServiceWrapper<T>>>
    : std::bool_constant<T::passive_adv> {};

template<typename T, typename = void>
struct is_active_adv_pred : std::false_type {};

template<typename T>
struct is_active_adv_pred<T, std::enable_if_t<ServiceWrapper<T>>>
    : std::bool_constant<T::active_adv> {};

// Config type detection traits (needed for SFINAE/template metaprogramming)
template<typename T>
struct is_advertising_config : std::bool_constant<AdvertisingConfigType<T>> {};

template<typename T>
struct is_connection_config : std::bool_constant<ConnectionConfigType<T>> {};

template<typename T>
struct is_security_config : std::bool_constant<SecurityConfigType<T>> {};

template<typename T>
struct is_server_callbacks_config : std::bool_constant<ServerCallbacksConfigType<T>> {};

template<typename T>
struct is_char_callbacks_config : std::bool_constant<CharCallbacksConfigType<T>> {};

// Helper to concatenate ServicesPack types
template<typename Pack1, typename Pack2>
struct concat_pack;

template<typename... S1, typename... S2>
struct concat_pack<ServicesPack<S1...>, ServicesPack<S2...>> {
    using type = ServicesPack<S1..., S2...>;
};

// Filter out config types and server callbacks to get only Services
template<typename...>
struct filter_non_config {
    using type = ServicesPack<>;  // Base case: empty pack
};

template<typename First, typename... Rest>
struct filter_non_config<First, Rest...> {
    using filtered_rest = typename filter_non_config<Rest...>::type;
    using type = std::conditional_t<
        (is_advertising_config<First>::value ||
         is_connection_config<First>::value ||
         is_security_config<First>::value ||
         is_server_callbacks_config<First>::value),
        filtered_rest,
        typename concat_pack<ServicesPack<First>, filtered_rest>::type
    >;
};

// Callback validation
template<typename T, auto CallbackFunc>
struct CallbackTraits {
    static constexpr bool is_valid_on_read = std::is_invocable_v<decltype(CallbackFunc), T&>;
    static constexpr bool is_valid_on_write = std::is_invocable_v<decltype(CallbackFunc), const T&>;
    static constexpr bool is_valid_on_status = std::is_invocable_v<decltype(CallbackFunc), int>;
    static constexpr bool is_valid_on_subscribe = std::is_invocable_v<decltype(CallbackFunc), uint16_t>;
};

} // namespace blex_core

// BLE Security IO Capabilities - must be declared before SecurityConfig forward declaration
enum BleIOCapability : uint8_t {
    DisplayOnly = 0,        // Can only display passkey
    DisplayYesNo = 1,       // Can display and confirm yes/no
    KeyboardOnly = 2,       // Can only input passkey
    NoInputNoOutput = 3,    // No input or output (Just Works pairing)
    KeyboardDisplay = 4     // Can both input and display passkey
};

// Forward declarations for config templates (needed by extract helpers)
template<int8_t, uint16_t, uint16_t, auto, const uint8_t*, size_t>
struct AdvertisingConfig;

template<uint16_t, uint16_t, uint16_t, uint16_t, uint16_t>
struct ConnectionConfig;

template<BleIOCapability, bool, bool, bool, uint32_t>
struct SecurityConfig;

template<auto, auto, auto>
struct ServerCallbacks;

template<auto, auto, auto, auto>
struct CharacteristicCallbacks;

namespace blex_core {

// Extract CharacteristicCallbacks from variadic args, or use default
template<typename...>
struct extract_char_callbacks {
    // Default: all callbacks nullptr
    using type = CharacteristicCallbacks<nullptr, nullptr, nullptr, nullptr>;
};

template<typename First, typename... Rest>
struct extract_char_callbacks<First, Rest...> {
    using type = std::conditional_t<
        is_char_callbacks_config<First>::value,
        First,
        typename extract_char_callbacks<Rest...>::type
    >;
};

// Helper to concatenate DescriptorsPack
template<typename Pack1, typename Pack2>
struct concat_descriptors;

template<typename... D1, typename... D2>
struct concat_descriptors<DescriptorsPack<D1...>, DescriptorsPack<D2...>> {
    using type = DescriptorsPack<D1..., D2...>;
};

// ---------------------- Hybrid Concept + Static Assert Validation ----------------------

// Validate that Args are either callback config or descriptor types
template<typename T>
struct is_valid_characteristic_arg : std::bool_constant<
    is_char_callbacks_config<T>::value ||
    // Accept any type as a potential descriptor (will be validated by descriptor traits later)
    !std::is_same_v<T, std::nullptr_t>  // Reject nullptr_t
> {};

// Macros for cleaner error message formatting
#define BLEX_ERROR_HEADER \
"\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n" \
"❌  BLEX Characteristic Argument Error\n" \
"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"

#define BLEX_ERROR_FOOTER \
"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"

// Individual argument validator with detailed static_assert - shows UUID, type, and position
template<auto UUID, typename T, size_t Index>
struct validate_single_arg {
    static_assert(
        is_valid_characteristic_arg<T>::value,
        BLEX_ERROR_HEADER
        "\n"
        "  ▸ LOOK ABOVE for: validate_single_arg<UUID, BAD_TYPE, INDEX>\n"
        "\n"
        "    UUID  = Characteristic UUID (identifies which characteristic has the error)\n"
        "    INDEX = 0-indexed position in Args (add 1 for actual position after Permissions)\n"
        "\n"
        "  Expected arguments:\n"
        "    • CharacteristicCallbacks<>::WithOnRead<>::WithOnWrite<>...\n"
        "    • UserDescription, PresentationFormat, AggregateFormat\n"
        "\n"
        "  ✓ Valid:   Characteristic<T, UUID, Permissions<...>, CharacteristicCallbacks<>...>\n"
        "  ✗ Invalid: std::nullptr_t, raw function pointers\n"
        "\n"
        BLEX_ERROR_FOOTER
    );
    static constexpr bool value = true;
};

// Hybrid concept: checks validity AND forces diagnostic on failure
// The OR trick: valid ? true : (instantiate_validator, false)
template<auto UUID, typename T, size_t Index>
concept ValidCharArgWithDiagnostic =
    is_valid_characteristic_arg<T>::value ||  // ← Fast path: if valid, done
    (validate_single_arg<UUID, T, Index>::value, false);  // ← Force static_assert, then fail

// Helper to validate all args with indices using the diagnostic concept
template<auto UUID, typename IndexSeq, typename... Args>
struct validate_all_args_with_diagnostic_impl;

template<auto UUID, size_t... Indices, typename... Args>
struct validate_all_args_with_diagnostic_impl<UUID, std::index_sequence<Indices...>, Args...> {
    static constexpr bool value = (ValidCharArgWithDiagnostic<UUID, Args, Indices> && ...);
};

// Concept that validates all args with diagnostics
template<auto UUID, typename... Args>
concept AllValidCharArgs = validate_all_args_with_diagnostic_impl<
    UUID,
    std::make_index_sequence<sizeof...(Args)>,
    Args...
>::value;

// Filter out callback configs to get only Descriptors
template<typename...>
struct filter_descriptors_from_args {
    using type = DescriptorsPack<>;
};

template<typename First, typename... Rest>
struct filter_descriptors_from_args<First, Rest...> {
    using filtered_rest = typename filter_descriptors_from_args<Rest...>::type;
    using type = std::conditional_t<
        is_char_callbacks_config<First>::value,
        filtered_rest,
        typename concat_descriptors<DescriptorsPack<First>, filtered_rest>::type
    >;
};


// Extract AdvertisingConfig from variadic args, or void if not provided
template<typename... /*Args*/>
struct extract_adv_config {
    using type = void;
};

template<typename First, typename... Rest>
struct extract_adv_config<First, Rest...> {
    using type = std::conditional_t<
        is_advertising_config<First>::value,
        First,
        typename extract_adv_config<Rest...>::type
    >;
};

// Extract ConnectionConfig from variadic args, or use default
template<typename...>
struct extract_conn_config {
    // Default
    using type = ConnectionConfig<0, 0, 0, 0, 0>;
};

template<typename First, typename... Rest>
struct extract_conn_config<First, Rest...> {
    using type = std::conditional_t<
        is_connection_config<First>::value,
        First,
        typename extract_conn_config<Rest...>::type
    >;
};

// Extract SecurityConfig from variadic args, or use default
template<typename...>
struct extract_security_config {
    // Default: NoInputNoOutput, no MITM, bonding enabled, secure connections enabled, no passkey
    using type = SecurityConfig<NoInputNoOutput, false, true, true, 0>;
};

template<typename First, typename... Rest>
struct extract_security_config<First, Rest...> {
    using type = std::conditional_t<
        is_security_config<First>::value,
        First,
        typename extract_security_config<Rest...>::type
    >;
};

// Extract ServerCallbacks from variadic args, or use default
template<typename...>
struct extract_server_callbacks {
    // Default: all callbacks nullptr
    using type = ServerCallbacks<nullptr, nullptr, nullptr>;
};

template<typename First, typename... Rest>
struct extract_server_callbacks<First, Rest...> {
    using type = std::conditional_t<
        is_server_callbacks_config<First>::value,
        First,
        typename extract_server_callbacks<Rest...>::type
    >;
};

} // namespace blex_core

// ---------------------- Public API Types ----------------------

// Internal permissions implementation (hidden from public API)
namespace detail {
    template<
        bool read,
        bool write,
        bool writeNoResponse,
        bool notify,
        bool indicate,
        uint8_t readSecurity,
        uint8_t writeSecurity,
        uint8_t writeNoRespSecurity,
        uint8_t notifySecurity,
        uint8_t indicateSecurity
    >
    struct PermissionsImpl {
        // Marker for trait detection
        using is_blex_permissions_tag = void;

        // Compile-time validation
        static_assert(readSecurity <= 3, "Read security level must be 0-3 (none/encrypt/auth/authorize)");
        static_assert(writeSecurity <= 3, "Write security level must be 0-3 (none/encrypt/auth/authorize)");
        static_assert(writeNoRespSecurity <= 3, "WriteNoResponse security level must be 0-3 (none/encrypt/auth/authorize)");
        static_assert(notifySecurity <= 3, "Notify security level must be 0-3 (none/encrypt/auth/authorize)");
        static_assert(indicateSecurity <= 3, "Indicate security level must be 0-3 (none/encrypt/auth/authorize)");

        // API for NimBLE integration
        static constexpr bool canRead = read;
        static constexpr bool canWrite = write;
        static constexpr bool canWriteNoResponse = writeNoResponse;
        static constexpr bool canNotify = notify;
        static constexpr bool canIndicate = indicate;

        // Derived security requirements (for NimBLE property flags)
        static constexpr bool requireEncryption =
            readSecurity >= 1 || writeSecurity >= 1 || writeNoRespSecurity >= 1 ||
            notifySecurity >= 1 || indicateSecurity >= 1;
        static constexpr bool requireAuthentication =
            readSecurity >= 2 || writeSecurity >= 2 || writeNoRespSecurity >= 2 ||
            notifySecurity >= 2 || indicateSecurity >= 2;
        static constexpr bool requireAuthorization =
            readSecurity >= 3 || writeSecurity >= 3 || writeNoRespSecurity >= 3 ||
            notifySecurity >= 3 || indicateSecurity >= 3;
    };
}

/**
 * @brief Compile-time fluent builder for BLE characteristic permissions with per-operation security.
 *
 * @details Provides a chainable API for defining BLE characteristic permissions with granular
 * security control per operation. Security levels follow BLE specification:
 * - Level 0: No security required
 * - Level 1: Encryption required
 * - Level 2: Authentication required (encrypted + authenticated pairing)
 * - Level 3: Authorization required (encrypted + authenticated + authorized)
 *
 * @par Basic Operations
 * - AllowRead - Enable read operation
 * - AllowWrite - Enable write operation
 * - AllowWriteNoResponse - Enable write without response
 * - AllowNotify - Enable notifications
 * - AllowIndicate - Enable indications
 *
 * @par Read with Security
 * - AllowEncryptedRead - Read requires encryption
 * - AllowAuthenticatedRead - Read requires authentication
 * - AllowAuthorizedRead - Read requires authorization
 *
 * @par Write with Security
 * - AllowEncryptedWrite - Write requires encryption
 * - AllowAuthenticatedWrite - Write requires authentication
 * - AllowAuthorizedWrite - Write requires authorization
 *
 * @par WriteNoResponse with Security
 * - AllowEncryptedWriteNoResponse
 * - AllowAuthenticatedWriteNoResponse
 * - AllowAuthorizedWriteNoResponse
 *
 * @par Notify with Security
 * - AllowEncryptedNotify
 * - AllowAuthenticatedNotify
 * - AllowAuthorizedNotify
 *
 * @par Indicate with Security
 * - AllowEncryptedIndicate
 * - AllowAuthenticatedIndicate
 * - AllowAuthorizedIndicate
 *
 * @par Examples
 * @code
 * // Read and notify with no security
 * using Perms1 = Permissions<>::AllowRead::AllowNotify;
 *
 * // Write requires authentication
 * using Perms2 = Permissions<>::AllowAuthenticatedWrite;
 *
 * // Read (no security), Write (encrypted), Notify
 * using Perms3 = Permissions<>::AllowRead::AllowEncryptedWrite::AllowNotify;
 * @endcode
 *
 * @note IDEs cannot autocomplete dependent template types (typename Blex::Permissions<>::)
 *       due to C++ language limitations. Hover over this type to see available builders.
 */
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
struct Permissions : detail::PermissionsImpl<read, write, writeNoResponse, notify, indicate,
                                              readSecurity, writeSecurity, writeNoRespSecurity,
                                              notifySecurity, indicateSecurity> {
    /// @brief Enable `read` operation
    using AllowRead = Permissions<true, write, writeNoResponse, notify, indicate, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `write`
    using AllowWrite = Permissions<read, true, writeNoResponse, notify, indicate, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `write without response`
    using AllowWriteNoResponse = Permissions<read, write, true, notify, indicate, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `notify`
    using AllowNotify = Permissions<read, write, writeNoResponse, true, indicate, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable  `indicate`
    using AllowIndicate = Permissions<read, write, writeNoResponse, notify, true, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;

    /// @brief Enable `read` with encryption required (security level 1)
    using AllowEncryptedRead = Permissions<true, write, writeNoResponse, notify, indicate, 1, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `read` with authentication required (security level 2)
    using AllowAuthenticatedRead = Permissions<true, write, writeNoResponse, notify, indicate, 2, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable  `read` with authorization required (security level 3)
    using AllowAuthorizedRead = Permissions<true, write, writeNoResponse, notify, indicate, 3, writeSecurity, writeNoRespSecurity, notifySecurity, indicateSecurity>;

    /// @brief Enable `write` with encryption required (security level 1)
    using AllowEncryptedWrite = Permissions<read, true, writeNoResponse, notify, indicate, readSecurity, 1, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `write` with authentication required (security level 2)
    using AllowAuthenticatedWrite = Permissions<read, true, writeNoResponse, notify, indicate, readSecurity, 2, writeNoRespSecurity, notifySecurity, indicateSecurity>;
    /// @brief Enable `write` with authorization required (security level 3)
    using AllowAuthorizedWrite = Permissions<read, true, writeNoResponse, notify, indicate, readSecurity, 3, writeNoRespSecurity, notifySecurity, indicateSecurity>;

    /// @brief Enable `write-no-response` with encryption required (security level 1)
    using AllowEncryptedWriteNoResponse = Permissions<read, write, true, notify, indicate, readSecurity, writeSecurity, 1, notifySecurity, indicateSecurity>;
    /// @brief Enable `write-no-response` with authentication required (security level 2)
    using AllowAuthenticatedWriteNoResponse = Permissions<read, write, true, notify, indicate, readSecurity, writeSecurity, 2, notifySecurity, indicateSecurity>;
    /// @brief Enable `write-no-response` with authorization required (security level 3)
    using AllowAuthorizedWriteNoResponse = Permissions<read, write, true, notify, indicate, readSecurity, writeSecurity, 3, notifySecurity, indicateSecurity>;

    /// @brief Enable `notify` with encryption required (security level 1)
    using AllowEncryptedNotify = Permissions<read, write, writeNoResponse, true, indicate, readSecurity, writeSecurity, writeNoRespSecurity, 1, indicateSecurity>;
    /// @brief Enable `notify` with authentication required (security level 2)
    using AllowAuthenticatedNotify = Permissions<read, write, writeNoResponse, true, indicate, readSecurity, writeSecurity, writeNoRespSecurity, 2, indicateSecurity>;
    /// @brief Enable `notify` with authorization required (security level 3)
    using AllowAuthorizedNotify = Permissions<read, write, writeNoResponse, true, indicate, readSecurity, writeSecurity, writeNoRespSecurity, 3, indicateSecurity>;

    /// @brief Enable `indicate` with encryption required (security level 1)
    using AllowEncryptedIndicate = Permissions<read, write, writeNoResponse, notify, true, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, 1>;
    /// @brief Enable `indicate` with authentication required (security level 2)
    using AllowAuthenticatedIndicate = Permissions<read, write, writeNoResponse, notify, true, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, 2>;
    /// @brief Enable `indicate` with authorization required (security level 3)
    using AllowAuthorizedIndicate = Permissions<read, write, writeNoResponse, notify, true, readSecurity, writeSecurity, writeNoRespSecurity, notifySecurity, 3>;
};

// ---------------------- Forward Declarations for Final Types ----------------------

// Forward-declare final descriptor types (defined in blex.hpp after backend specializations)
template<typename T, auto UUID, T Value, typename Perms>
struct ConstDescriptor;

// ---------------------- Descriptor and Characteristic Templates ----------------------

/**
 * @brief Descriptor base - pure metadata, backend-agnostic
 * @note CRTP pattern: Derived=void means self-reference
 */
template<
    typename T,
    auto UUID,
    typename Perms = Permissions<>::AllowRead,
    size_t MaxSize = sizeof(T),
    typename Derived = void  // void = self-reference
>
struct DescriptorBase {
    static constexpr auto _ = (blex_core::check_uuid_type<decltype(UUID)>(), 0);

    using Base = std::conditional_t<std::is_void_v<Derived>, DescriptorBase, Derived>;  // CRTP: void → self, otherwise → Derived
    using value_type = T;
    static constexpr auto uuid = UUID;
    using perms_type = Perms;
    static constexpr size_t max_size = MaxSize;
};

// ---------------------- Backend Trait Forward Declarations ----------------------

/**
 * @brief Backend trait for Descriptors (specialized per backend)
 * @details Provides backend-specific descriptor registration
 */
template<typename DescriptorBase>
struct DescriptorBackend;

/**
 * @brief Backend trait for Characteristic (specialized per backend)
 * @details Provides backend-specific storage and operations
 * Specialized in nimble.hpp, bluez.hpp, etc.
 */
template<typename CharacteristicBase>
struct CharacteristicBackend;

/**
 * @brief Backend trait for Service (specialized per backend)
 * @details Provides backend-specific service registration
 * Specialized in nimble.hpp, bluez.hpp, etc.
 */
template<typename ServiceBase>
struct ServiceBackend;

// ---------------------- Characteristic Base (Backend-Agnostic) ----------------------

/**
 * @brief Backend-agnostic characteristic type definition
 * @details Contains compile-time metadata, callbacks, permissions - no NimBLE dependency
 *
 * @tparam T Value type
 * @tparam UUID Characteristic UUID (uint16_t or const char*)
 * @tparam Perms Permissions<...> type from builder
 * @tparam Args Variadic: CharacteristicCallbacks, Descriptors
 */
template<
    typename T,
    auto UUID,
    typename Perms,
    typename Derived = void,
    typename... Args
> requires blex_core::AllValidCharArgs<UUID, Args...>
struct CharacteristicBase {
    using Base = std::conditional_t<std::is_void_v<Derived>, CharacteristicBase, Derived>;

    static constexpr auto _ = (blex_core::check_uuid_type<decltype(UUID)>(), 0);

    // Extract CharacteristicCallbacks config from Args (or use default with all nullptr)
    using CallbacksConfig = typename blex_core::extract_char_callbacks<Args...>::type;

    // Extract callbacks directly from config
    using ReadHandlerType = decltype(CallbacksConfig::on_read);
    using WriteHandlerType = decltype(CallbacksConfig::on_write);
    using StatusHandlerType = decltype(CallbacksConfig::on_status);
    using SubscribeHandlerType = decltype(CallbacksConfig::on_subscribe);

    static constexpr ReadHandlerType ReadHandler = CallbacksConfig::on_read;
    static constexpr WriteHandlerType WriteHandler = CallbacksConfig::on_write;
    static constexpr StatusHandlerType StatusHandler = CallbacksConfig::on_status;
    static constexpr SubscribeHandlerType SubscribeHandler = CallbacksConfig::on_subscribe;

    // Consolidated callback validation - validates all callbacks in one place (DRY principle)
    static_assert(ReadHandler == nullptr ||
                  (blex_core::CallbackTraits<T, ReadHandler>::is_valid_on_read && Perms::canRead),
                  "Invalid BLE OnRead callback: must be invocable with 'T&' and characteristic must allow Read");

    static_assert(WriteHandler == nullptr ||
                  (blex_core::CallbackTraits<T, WriteHandler>::is_valid_on_write && (Perms::canWrite || Perms::canWriteNoResponse)),
                  "Invalid BLE OnWrite callback: must be invocable with 'const T&' and characteristic must allow Write or WriteNoResponse");

    static_assert(SubscribeHandler == nullptr ||
                  (blex_core::CallbackTraits<T, SubscribeHandler>::is_valid_on_subscribe && Perms::canNotify),
                  "Invalid BLE OnSubscribe callback: must be invocable with 'uint16_t' and characteristic must allow Notifications");

    static_assert(StatusHandler == nullptr ||
                  blex_core::CallbackTraits<T, StatusHandler>::is_valid_on_status,
                  "Invalid BLE OnStatus callback: must be invocable with 'int'");


    // Type metadata (backend-agnostic)
    using value_type = T;
    static constexpr auto uuid = UUID;
    using perms_type = Perms;
    using descriptors_pack = typename blex_core::filter_descriptors_from_args<Args...>::type;
    static constexpr bool is_const_characteristic = false;

    // Validation hook for descriptors
    static constexpr void validate_all_descriptors() {}
};

// ---------------------- Service Base (Backend-Agnostic) ----------------------

/**
 * @brief Backend-agnostic service type definition
 * @details Contains compile-time metadata: UUID and characteristics pack - no NimBLE dependency
 *
 * @tparam UUID Service UUID (uint16_t or const char*)
 * @tparam Derived CRTP parameter for automatic Base typedef (void = self-reference)
 * @tparam Chars Variadic pack of Characteristic types
 */
template<auto UUID, typename Derived = void, typename... Chars>
struct ServiceBase {
    static constexpr auto _ = (blex_core::check_uuid_type<decltype(UUID)>(), 0);

    using Base = std::conditional_t<std::is_void_v<Derived>, ServiceBase, Derived>;  // CRTP
    static constexpr auto uuid = UUID;
    using chars_pack = blex_core::CharsPack<Chars...>;

    // Validate all characteristics' descriptors at compile-time
    static constexpr void validate() {
        (Chars::validate_all_descriptors(), ...);
    }
};

// ---------------------- Configuration Templates ----------------------

// AdvertisingConfig: Compile-time fluent builder for advertising configuration
template<
    int8_t TxPower = 0,
    uint16_t IntervalMin = 100,
    uint16_t IntervalMax = 150,
    auto Appearance = 0,  // 0 = BleAppearance::kUnknown (unknown device type)
    const uint8_t* ManufacturerData = nullptr,
    size_t ManufacturerDataLen = 0
>
struct AdvertisingConfig {
    // Validate Appearance type and range FIRST (before using it)
    static_assert(std::is_same_v<decltype(Appearance), blex_standard::BleAppearance> ||
                  (std::is_integral_v<decltype(Appearance)> &&
                   static_cast<uint64_t>(Appearance) <= 0xFFFF),
                  "Appearance must be blex_standard::BleAppearance enum or uint16_t value [0x0000-0xFFFF]. "
                  "Example: blex_standard::BleAppearance::kGenericSensor or 0x0540");

    // Marker for trait detection
    using is_blex_advertising_config_tag = void;

    // Compile-time configuration (user-specified via template parameters)
    static constexpr int8_t default_tx_power = TxPower;
    static constexpr uint16_t default_adv_interval_min = IntervalMin;
    static constexpr uint16_t default_adv_interval_max = IntervalMax;
    static constexpr uint16_t default_appearance = static_cast<uint16_t>(Appearance);
    static constexpr uint8_t default_flags = 0x06;  // LE General Discoverable + BR/EDR Not Supported
    static constexpr const uint8_t* manufacturer_data = ManufacturerData;
    static constexpr size_t manufacturer_data_len = ManufacturerDataLen;

    // ESP32-S3 TX power range (validation)
    static constexpr int8_t min_tx_power = -12;  // dBm
    static constexpr int8_t max_tx_power = 9;    // dBm

    // BLE spec advertising interval range (validation)
    static constexpr uint16_t min_adv_interval = 20;      // ms
    static constexpr uint16_t max_adv_interval = 10240;   // ms

    // Compile-time validation
    static_assert(TxPower >= min_tx_power && TxPower <= max_tx_power,
                 "TX power must be in range [-12, 9] dBm");
    static_assert(IntervalMin >= min_adv_interval && IntervalMin <= max_adv_interval,
                 "Advertising interval min must be in range [20, 10240] ms");
    static_assert(IntervalMax >= min_adv_interval && IntervalMax <= max_adv_interval,
                 "Advertising interval max must be in range [20, 10240] ms");
    static_assert(IntervalMin <= IntervalMax,
                 "Advertising interval min must be <= max");
    static_assert(ManufacturerDataLen <= 27, "Manufacturer data length must be <= 27 bytes");

    // Fluent builder methods - each returns a new type with updated config
    template<int8_t NewTxPower>
    using WithTxPower = AdvertisingConfig<NewTxPower, IntervalMin, IntervalMax, Appearance, ManufacturerData, ManufacturerDataLen>;

    template<uint16_t NewMin, uint16_t NewMax>
    using WithIntervals = AdvertisingConfig<TxPower, NewMin, NewMax, Appearance, ManufacturerData, ManufacturerDataLen>;

    template<auto NewAppearance>
    using WithAppearance = AdvertisingConfig<TxPower, IntervalMin, IntervalMax, NewAppearance, ManufacturerData, ManufacturerDataLen>;

    // Deduce size from array reference automatically
    // Usage: ::WithManufacturerData<mfg_data>
    template<const auto& Data>
    using WithManufacturerData = AdvertisingConfig<TxPower, IntervalMin, IntervalMax, Appearance, Data, sizeof(Data)>;
};

// Internal helpers for BLE unit conversions (hidden from public API)
namespace detail {
    // Convert milliseconds to BLE connection interval units (1.25ms per unit)
    constexpr uint16_t ms_to_interval_units(uint16_t ms) {
        // ms / 1.25 = ms * 4 / 5, with rounding to the nearest
        return static_cast<uint16_t>((static_cast<uint32_t>(ms) * 4 + 2) / 5);
    }

    // Convert milliseconds to BLE timeout units (10ms per unit)
    constexpr uint16_t ms_to_timeout_units(uint16_t ms) {
        // ms / 10, with rounding to nearest
        return static_cast<uint16_t>((static_cast<uint32_t>(ms) + 5) / 10);
    }
}

// ConnectionConfig: Compile-time connection configuration
// Note: Intervals and timeouts are specified in milliseconds
template<
    uint16_t MTU = 247,
    uint16_t ConnIntervalMinMs = 15,      // in milliseconds
    uint16_t ConnIntervalMaxMs = 15,      // in milliseconds
    uint16_t ConnLatency = 0,
    uint16_t SupervisionTimeoutMs = 4000  // in milliseconds
>
struct ConnectionConfig {
    // Marker for trait detection
    using is_blex_connection_config_tag = void;

    // Compile-time configuration in BLE spec units (for NimBLE API)
    static constexpr uint16_t mtu = MTU;
    static constexpr uint16_t conn_interval_min = detail::ms_to_interval_units(ConnIntervalMinMs);
    static constexpr uint16_t conn_interval_max = detail::ms_to_interval_units(ConnIntervalMaxMs);
    static constexpr uint16_t conn_latency = ConnLatency;
    static constexpr uint16_t supervision_timeout = detail::ms_to_timeout_units(SupervisionTimeoutMs);

    // User-facing values in milliseconds (for display/documentation)
    static constexpr uint16_t conn_interval_min_ms = ConnIntervalMinMs;
    static constexpr uint16_t conn_interval_max_ms = ConnIntervalMaxMs;
    static constexpr uint16_t supervision_timeout_ms = SupervisionTimeoutMs;

    // BLE spec connection parameter ranges (in milliseconds for validation)
    static constexpr uint16_t min_mtu = 23;         // BLE minimum
    static constexpr uint16_t max_mtu = 517;        // BLE maximum
    static constexpr uint16_t min_conn_interval_ms = 8;     // 7.5ms rounded up
    static constexpr uint16_t max_conn_interval_ms = 4000;
    static constexpr uint16_t max_conn_latency = 499;
    static constexpr uint16_t min_supervision_timeout_ms = 100;
    static constexpr uint16_t max_supervision_timeout_ms = 32000;

    // Compile-time validation (on millisecond values)
    static_assert(MTU >= min_mtu && MTU <= max_mtu,
                 "MTU must be in range [23, 517] bytes");
    static_assert(ConnIntervalMinMs >= min_conn_interval_ms && ConnIntervalMinMs <= max_conn_interval_ms,
                 "Connection interval min must be in range [8, 4000] milliseconds");
    static_assert(ConnIntervalMaxMs >= min_conn_interval_ms && ConnIntervalMaxMs <= max_conn_interval_ms,
                 "Connection interval max must be in range [8, 4000] milliseconds");
    static_assert(ConnIntervalMinMs <= ConnIntervalMaxMs,
                 "Connection interval min must be <= max");
    static_assert(ConnLatency <= max_conn_latency,
                 "Connection latency must be <= 499");
    static_assert(SupervisionTimeoutMs >= min_supervision_timeout_ms && SupervisionTimeoutMs <= max_supervision_timeout_ms,
                 "Supervision timeout must be in range [100, 32000] milliseconds");

    // Fluent builder methods - each returns a new type with updated config
    template<uint16_t NewMTU>
    using WithMTU = ConnectionConfig<NewMTU, ConnIntervalMinMs, ConnIntervalMaxMs, ConnLatency, SupervisionTimeoutMs>;

    template<uint16_t NewMinMs, uint16_t NewMaxMs>
    using WithIntervals = ConnectionConfig<MTU, NewMinMs, NewMaxMs, ConnLatency, SupervisionTimeoutMs>;

    template<uint16_t NewLatency>
    using WithLatency = ConnectionConfig<MTU, ConnIntervalMinMs, ConnIntervalMaxMs, NewLatency, SupervisionTimeoutMs>;

    template<uint16_t NewTimeoutMs>
    using WithTimeout = ConnectionConfig<MTU, ConnIntervalMinMs, ConnIntervalMaxMs, ConnLatency, NewTimeoutMs>;
};

// SecurityConfig: Compile-time BLE security and pairing configuration
template<
    BleIOCapability IOCapabilities = NoInputNoOutput,
    bool MITMProtection = false,
    bool Bonding = true,
    bool SecureConnections = true,
    uint32_t Passkey = 0
>
struct SecurityConfig {
    // Marker for trait detection
    using is_blex_security_config_tag = void;

    // Compile-time configuration
    static constexpr BleIOCapability io_capabilities = IOCapabilities;
    static constexpr bool mitm_protection = MITMProtection;
    static constexpr bool bonding = Bonding;
    static constexpr bool secure_connections = SecureConnections;
    static constexpr uint32_t passkey = Passkey;

    // Compile-time validation
    static_assert(Passkey <= 999999, "BLE passkey must be 6 digits (0-999999)");

    // Fluent builder methods for configuration
    template<BleIOCapability NewIOCap>
    using WithIOCapabilities = SecurityConfig<NewIOCap, MITMProtection, Bonding, SecureConnections, Passkey>;

    template<bool NewMITM>
    using WithMITM = SecurityConfig<IOCapabilities, NewMITM, Bonding, SecureConnections, Passkey>;

    template<bool NewBonding>
    using WithBonding = SecurityConfig<IOCapabilities, MITMProtection, NewBonding, SecureConnections, Passkey>;

    template<bool NewSecureConn>
    using WithSecureConnections = SecurityConfig<IOCapabilities, MITMProtection, Bonding, NewSecureConn, Passkey>;

    template<uint32_t NewPasskey>
    using WithPasskey = SecurityConfig<IOCapabilities, MITMProtection, Bonding, SecureConnections, NewPasskey>;
};

// Internal implementation details for ServerCallbacks (hidden from user-facing API)
namespace server_callbacks_detail {
    template<auto Cb, auto OnConnectCb, auto OnDisconnectCb, auto OnMTUChangeCb>
    struct WithOnConnectImpl {
        // Backend validates callback signature at compile time when invoking
        using type = ServerCallbacks<Cb, OnDisconnectCb, OnMTUChangeCb>;
    };

    template<auto Cb, auto OnConnectCb, auto OnDisconnectCb, auto OnMTUChangeCb>
    struct WithOnDisconnectImpl {
        // Backend validates callback signature at compile time when invoking
        using type = ServerCallbacks<OnConnectCb, Cb, OnMTUChangeCb>;
    };

    template<auto Cb, auto OnConnectCb, auto OnDisconnectCb, auto OnMTUChangeCb>
    struct WithOnMTUChangeImpl {
        // Backend validates callback signature at compile time when invoking
        using type = ServerCallbacks<OnConnectCb, OnDisconnectCb, Cb>;
    };
} // namespace server_callbacks_detail

// ServerCallbacks: Compile-time fluent builder for server callbacks
template<
    auto OnConnectCb = nullptr,
    auto OnDisconnectCb = nullptr,
    auto OnMTUChangeCb = nullptr
>
struct ServerCallbacks {
    // Marker for trait detection
    using is_blex_server_callbacks_config_tag = void;

    // Compile-time configuration
    static constexpr auto on_connect = OnConnectCb;
    static constexpr auto on_disconnect = OnDisconnectCb;
    static constexpr auto on_mtu_change = OnMTUChangeCb;

    // Fluent builder methods - each validates signature and returns new type
    template<auto NewCallback>
    using WithOnConnect = typename server_callbacks_detail::WithOnConnectImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;

    template<auto NewCallback>
    using WithOnDisconnect = typename server_callbacks_detail::WithOnDisconnectImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;

    template<auto NewCallback>
    using WithOnMTUChange = typename server_callbacks_detail::WithOnMTUChangeImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;
};

// Internal implementation details for CharacteristicCallbacks (hidden from user-facing API)
namespace char_callbacks_detail {
    // Simple implementation: no validation at the builder level, validation happens in the Characteristic template
    template<auto Cb, auto OnReadCb, auto OnWriteCb, auto OnStatusCb, auto OnSubscribeCb>
    struct WithOnReadImpl {
        using type = CharacteristicCallbacks<Cb, OnWriteCb, OnStatusCb, OnSubscribeCb>;
    };

    template<auto Cb, auto OnReadCb, auto OnWriteCb, auto OnStatusCb, auto OnSubscribeCb>
    struct WithOnWriteImpl {
        using type = CharacteristicCallbacks<OnReadCb, Cb, OnStatusCb, OnSubscribeCb>;
    };

    template<auto Cb, auto OnReadCb, auto OnWriteCb, auto OnStatusCb, auto OnSubscribeCb>
    struct WithOnStatusImpl {
        using type = CharacteristicCallbacks<OnReadCb, OnWriteCb, Cb, OnSubscribeCb>;
    };

    template<auto Cb, auto OnReadCb, auto OnWriteCb, auto OnStatusCb, auto OnSubscribeCb>
    struct WithOnSubscribeImpl {
        using type = CharacteristicCallbacks<OnReadCb, OnWriteCb, OnStatusCb, Cb>;
    };
} // namespace char_callbacks_detail

// CharacteristicCallbacks: Compile-time fluent builder for characteristic callbacks
template<
    auto OnReadCb = nullptr,
    auto OnWriteCb = nullptr,
    auto OnStatusCb = nullptr,
    auto OnSubscribeCb = nullptr
>
struct CharacteristicCallbacks {
    // Marker for trait detection
    using is_blex_char_callbacks_config_tag = void;

    // Compile-time configuration
    static constexpr auto on_read = OnReadCb;
    static constexpr auto on_write = OnWriteCb;
    static constexpr auto on_status = OnStatusCb;
    static constexpr auto on_subscribe = OnSubscribeCb;

    // Fluent builder methods - each returns a new type with an updated callback
    template<auto NewCallback>
    using WithOnRead = typename char_callbacks_detail::WithOnReadImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnWrite = typename char_callbacks_detail::WithOnWriteImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnStatus = typename char_callbacks_detail::WithOnStatusImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnSubscribe = typename char_callbacks_detail::WithOnSubscribeImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;
};

// ---------------------- Server Base (Backend-Agnostic) ----------------------

/**
 * @brief Backend-agnostic server base type
 * @details Compile-time metadata: device name, services, advertising/connection config
 * Uses CRTP for automatic Base typedef - defaults to self-reference (void = self).
 * Contains no runtime state - all metadata is extracted from template parameters.
 */
template<
    const char* DeviceName,
    const char* ShortName,
    typename Derived = void,
    typename... Args
>
struct ServerBase {
    using Base = std::conditional_t<std::is_void_v<Derived>, ServerBase, Derived>;  // CRTP

    // Device identity
    static constexpr const char* device_name = DeviceName;
    static constexpr const char* short_name = ShortName;

    // Extract configurations from variadic Args
    using AdvConfig = typename blex_core::extract_adv_config<Args...>::type;
    using ConnConfig = typename blex_core::extract_conn_config<Args...>::type;
    using SecurityConfig = typename blex_core::extract_security_config<Args...>::type;
    using CallbacksConfig = typename blex_core::extract_server_callbacks<Args...>::type;
    using ServicesTuple = typename blex_core::filter_non_config<Args...>::type;

    // Extract callbacks from ServerCallbacks config
    static constexpr auto ConnectHandler = CallbacksConfig::on_connect;
    static constexpr auto DisconnectHandler = CallbacksConfig::on_disconnect;
    static constexpr auto MTUChangeHandler = CallbacksConfig::on_mtu_change;

    // Advertising service filtering (compile-time)
    using PassiveServices = typename blex_core::filter_services_pack<blex_core::is_passive_adv_pred, ServicesTuple>::type;
    using ActiveServices = typename blex_core::filter_services_pack<blex_core::is_active_adv_pred, ServicesTuple>::type;
};

// ---------------------- ServerBackend Trait (Forward Declaration) ----------------------

/**
 * @brief Server backend trait (specialized per BLE stack)
 * @details Provides runtime server operations (init, advertising, connections)
 */
template<typename ServerBaseT>
struct ServerBackend;

#endif // BLEX_CORE_HPP_