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
 * - Fluent builder configuration types (Permissions, AdvertisementConfig, etc.)
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

// ---------------------- Type Utilities ----------------------

/// @brief C++20 std::remove_cvref_t replacement for older compilers
template<typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

// ---------------------- C++20 Concepts ----------------------

/// @brief UUID type validation concept
/// @details Accepts integral types (uint16_t), char pointers (char*), or char arrays (char[])
template<typename T>
concept UuidType = requires {
    requires std::is_integral_v<remove_cvref_t<T>> ||
             std::is_same_v<remove_cvref_t<std::remove_pointer_t<T>>, char> ||
             std::is_same_v<remove_cvref_t<std::remove_extent_t<T>>, char>;
};

// Service wrapper concept
template<typename T>
concept ServiceWrapper = requires { typename T::service_type; };

// Advertisement config concept
template<typename T>
concept AdvertisementConfigType = requires { typename T::is_blex_advertisement_config_tag; };

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
static constexpr void check_uuid_type() noexcept {
    static_assert(UuidType<T>, "UUID must be an integer (e.g., uint16_t) or a C string (char*/char[])");
}

// Compile-time string length helper
static constexpr size_t const_strlen(const char* str) noexcept {
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
    using type = T::service_type;
};

template<typename T>
using unwrap_service = unwrap_service_impl<T>::type;

/// @brief Type aliases for template parameter packs using std::tuple
/// @details Eliminates custom pack types - std::tuple provides identical functionality
///          with better consistency and no forced conversions between pack types
template<typename... Services>
using ServicesPack = std::tuple<Services...>;

template<typename... Chars>
using CharsPack = std::tuple<Chars...>;

template<typename... Descs>
using DescriptorsPack = std::tuple<Descs...>;

// Predicates for filtering (concept-based with SFINAE fallback for member access)
template<typename, typename = void>
struct is_passive_adv_pred : std::false_type {};

template<typename T>
struct is_passive_adv_pred<T, std::enable_if_t<ServiceWrapper<T>>>
    : std::bool_constant<T::passive_adv> {};

template<typename, typename = void>
struct is_active_adv_pred : std::false_type {};

template<typename T>
struct is_active_adv_pred<T, std::enable_if_t<ServiceWrapper<T>>>
    : std::bool_constant<T::active_adv> {};

/// @brief Generic pack concatenation (works with std::tuple and type aliases)
/// @details Concatenates two tuple-like types into a single tuple
template<typename Pack1, typename Pack2>
struct concat_packs;

template<typename... T1, typename... T2>
struct concat_packs<std::tuple<T1...>, std::tuple<T2...>> {
    using type = std::tuple<T1..., T2...>;
};

// ---------------------- Generic Filter (DRY principle) ----------------------

/**
 * @brief Generic type filter: keeps or excludes types based on a predicate
 * @tparam Predicate Template template parameter - trait with ::value member
 * @tparam ResultPack Pack type to return (std::tuple, ServicesPack, DescriptorsPack, etc.)
 * @tparam IncludeOnMatch true = keep matching types, false = exclude matching types
 * @tparam Types Variadic pack to filter
 */
template<template<typename> class Predicate, template<typename...> class ResultPack, bool IncludeOnMatch, typename...>
struct filter_types {
    using type = ResultPack<>;  // Base case: empty pack
};

template<template<typename> class Predicate, template<typename...> class ResultPack, bool IncludeOnMatch, typename First, typename... Rest>
struct filter_types<Predicate, ResultPack, IncludeOnMatch, First, Rest...> {
    using filtered_rest = typename filter_types<Predicate, ResultPack, IncludeOnMatch, Rest...>::type;
    using type = std::conditional_t<
        Predicate<First>::value == IncludeOnMatch,
        typename concat_packs<ResultPack<First>, filtered_rest>::type,
        filtered_rest
    >;
};

/// @brief Predicate: detects any config type (advertisement, connection, security, server callbacks)
template<typename T>
struct is_any_config : std::bool_constant<
    AdvertisementConfigType<T> ||
    ConnectionConfigType<T> ||
    SecurityConfigType<T> ||
    ServerCallbacksConfigType<T>
> {};

// ---------------------- Specialized Filters (using generic pattern) ----------------------

/// @brief Service filtering (returns a tuple for advertising configuration)
template<template<typename> class Predicate, typename... Services>
using filter_services = filter_types<Predicate, std::tuple, true, Services...>;

/// @brief Apply filter_services to ServicesPack
template<template<typename> class Predicate, typename SvcPack>
struct filter_services_pack;

template<template<typename> class Predicate, typename... Services>
struct filter_services_pack<Predicate, ServicesPack<Services...>> {
    using type = typename filter_types<Predicate, std::tuple, true, Services...>::type;
};

/// @brief Filter out config types and server callbacks to get only Services
template<typename... Args>
using filter_non_config = filter_types<is_any_config, std::tuple, false, Args...>;

// Callback validation
template<typename T, auto CallbackFunc>
struct CallbackTraits {
    static constexpr bool is_valid_on_read = std::is_invocable_v<decltype(CallbackFunc), T&>;
    static constexpr bool is_valid_on_write =
        // Standard: callback(const T&)
        std::is_invocable_v<decltype(CallbackFunc), const T&> ||
        (std::is_array_v<T> &&
        // callback(const uint8_t*, size_t) for uint8_t[N] types
         std::is_same_v<std::remove_extent_t<T>, uint8_t> &&
         std::is_invocable_v<decltype(CallbackFunc), const uint8_t*, size_t>);
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
template<int8_t, uint16_t, uint16_t, auto, const auto&, const char*>
struct AdvertisementConfig;

template<uint16_t, uint16_t, uint16_t, uint16_t, uint16_t>
struct ConnectionConfig;

template<BleIOCapability, bool, bool, bool, uint32_t>
struct SecurityConfig;

template<auto, auto, auto>
struct ServerCallbacks;

template<auto, auto, auto, auto>
struct CharacteristicCallbacks;

namespace blex_core {

// ---------------------- Generic Extractor (DRY principle) ----------------------

/**
 * @brief Generic extractor: finds first type matching Predicate, or returns Default
 * @tparam Predicate Template template parameter - trait with ::value member
 * @tparam Default Type to return if no match found
 * @tparam Args Variadic pack to search
 */
template<template<typename> class Predicate, typename Default, typename... Args>
struct extract_first_matching {
    using type = Default;
};

template<template<typename> class Predicate, typename Default, typename First, typename... Rest>
struct extract_first_matching<Predicate, Default, First, Rest...> {
    using type = std::conditional_t<
        Predicate<First>::value,
        First,
        typename extract_first_matching<Predicate, Default, Rest...>::type
    >;
};

// ---------------------- Specialized Extractors (using generic pattern) ----------------------

/// @brief Predicate wrapper for CharCallbacksConfigType concept (for use in metaprogramming)
template<typename T>
struct is_char_callbacks_pred : std::bool_constant<CharCallbacksConfigType<T>> {};

/// @brief Extract CharacteristicCallbacks from variadic args, or use default
template<typename... Args>
using extract_char_callbacks = extract_first_matching<
    is_char_callbacks_pred,
    CharacteristicCallbacks<nullptr, nullptr, nullptr, nullptr>,
    Args...
>;

// ---------------------- Hybrid Concept + Static Assert Validation ----------------------

/// @brief Validate that Args are either callback config or descriptor types
template<typename T>
struct is_valid_characteristic_arg : std::bool_constant<
    CharCallbacksConfigType<T> ||
    // Accept any type as a potential descriptor (will be validated by descriptor traits later)
    !std::is_same_v<T, std::nullptr_t>  // Reject nullptr_t
> {};

/// @brief Macros for consistent error message formatting across BLEX
/// @details Provides visual separators and headers for static_assert compile-time errors
#define BLEX_ERROR_SEP_LONG "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
#define BLEX_ERROR_SEP_SHORT "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
#define BLEX_ERROR_HEADER(title, sep) "\n" sep "\n" "❌  " title "\n" sep "\n"
#define BLEX_ERROR_FOOTER(sep) sep "\n"

// Individual argument validator with detailed static_assert - shows UUID, type, and position
template<auto UUID, typename T, size_t Index>
struct validate_single_arg {
    static_assert(
        is_valid_characteristic_arg<T>::value,
        BLEX_ERROR_HEADER("BLEX Characteristic Argument Error", BLEX_ERROR_SEP_LONG)
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
        BLEX_ERROR_FOOTER(BLEX_ERROR_SEP_LONG)
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

/// @brief Filter out callback configs to get only Descriptors
template<typename... Args>
using filter_descriptors_from_args = filter_types<is_char_callbacks_pred, std::tuple, false, Args...>;

/// @brief Predicate wrapper for AdvertisementConfigType concept (for use in metaprogramming)
template<typename T>
struct is_advertisement_pred : std::bool_constant<AdvertisementConfigType<T>> {};

/// @brief Predicate wrapper for ConnectionConfigType concept (for use in metaprogramming)
template<typename T>
struct is_connection_pred : std::bool_constant<ConnectionConfigType<T>> {};

/// @brief Predicate wrapper for SecurityConfigType concept (for use in metaprogramming)
template<typename T>
struct is_security_pred : std::bool_constant<SecurityConfigType<T>> {};

/// @brief Predicate wrapper for ServerCallbacksConfigType concept (for use in metaprogramming)
template<typename T>
struct is_server_callbacks_pred : std::bool_constant<ServerCallbacksConfigType<T>> {};

/// @brief Unwrap AdvertisementConfig::type member if it exists (for builder support)
/// @details If T has a nested `type` member, returns T::type; otherwise returns T
template<typename T, typename = void>
struct unwrap_adv_config_impl {
    using type = T;
};

template<typename T>
struct unwrap_adv_config_impl<T, std::void_t<typename T::type>> {
    using type = T::type;
};

template<typename T>
using unwrap_adv_config = unwrap_adv_config_impl<T>::type;

/// @brief Extract AdvertisementConfig from variadic args, or void if not provided
/// @details Automatically unwraps Builder::type to final AdvertisementConfig
template<typename...>
struct extract_adv_config {
    using type = void;
};

template<typename First, typename... Rest>
struct extract_adv_config<First, Rest...> {
    using type = unwrap_adv_config<
        std::conditional_t<
            AdvertisementConfigType<First>,
            First,
            typename extract_adv_config<Rest...>::type
        >
    >;
};

/// @brief Extract ConnectionConfig from variadic args, or void if not provided
template<typename... Args>
using extract_conn_config = extract_first_matching<
    is_connection_pred,
    void,
    Args...
>;

/// @brief Extract SecurityConfig from variadic args, or void if not provided
template<typename... Args>
using extract_security_config = extract_first_matching<
    is_security_pred,
    void,
    Args...
>;

/// @brief Extract ServerCallbacks from variadic args, or default (all nullptr) if not provided
template<typename... Args>
using extract_server_callbacks = extract_first_matching<
    is_server_callbacks_pred,
    ServerCallbacks<nullptr, nullptr, nullptr>,
    Args...
>;

} // namespace blex_core

// ---------------------- Public API Types ----------------------

// Internal permissions implementation (hidden from public API)
namespace detail {
    /**
     * @brief Security permission levels for BLE GATT operations
     *
     * @details Defines security requirements for BLE GATT operations:
     * - Disabled: Operation not allowed
     * - Unprotected: Operation allowed with no security requirements
     * - Encrypted: Operation requires encrypted connection
     * - Authenticated: Operation requires authenticated pairing (encrypted + authenticated)
     * - Authorized: Operation requires authorization (encrypted + authenticated + authorized)
     *
     * @note Values 0-4 are intentionally chosen to fit in 3 bits for efficient packing
     */
    enum class SecPerm : uint8_t {
        Disabled = 0,        // Operation disabled
        Unprotected = 1,     // Allowed, no security required
        Encrypted = 2,       // Requires encryption
        Authenticated = 3,   // Requires authentication
        Authorized = 4       // Requires authorization
    };

    template<
        SecPerm read,
        SecPerm write,
        bool writeNoResp,
        bool notify,
        bool indicate,
        bool broadcast
    >
    struct PermissionsImpl {
        // Marker for trait detection
        using is_blex_permissions_tag = void;

        // Compile-time validation
        static_assert(read <= SecPerm::Authorized, "Invalid read security level");
        static_assert(write <= SecPerm::Authorized, "Invalid write security level");

        // API for NimBLE integration - operation enabled flags
        static constexpr bool canRead = read > SecPerm::Disabled;
        static constexpr bool canWrite = write > SecPerm::Disabled;
        static constexpr bool canWriteNoResponse = writeNoResp;
        static constexpr bool canNotify = notify;
        static constexpr bool canIndicate = indicate;
        static constexpr bool canBroadcast = broadcast;

        // Per-operation security levels (exposed for nimble.hpp)
        // Convert SecPerm enum to numeric security level (0-4)
        static constexpr uint8_t read_security = static_cast<uint8_t>(read);
        static constexpr uint8_t write_security = static_cast<uint8_t>(write);
    };


} // namespace detail

/**
 * @brief Compile-time fluent builder for BLE characteristic permissions with per-operation security.
 *
 * @details Provides a chainable API for defining BLE characteristic permissions with granular
 * security control per operation. Security levels follow BLE specification:
 * - Disabled: Operation not allowed
 * - Unprotected: Operation allowed with no security
 * - Encrypted: Operation requires encryption
 * - Authenticated: Operation requires authentication (encrypted + authenticated pairing)
 * - Authorized: Operation requires authorization (encrypted + authenticated + authorized)
 *
 * @par Basic Operations
 * - AllowRead - Enable read operation (no security)
 * - AllowWrite - Enable write operation (no security)
 * - AllowWriteNoResponse - Enable write without response
 * - AllowNotify - Enable notifications
 * - AllowIndicate - Enable indications
 * - AllowBroadcast - Enable broadcast
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
    detail::SecPerm read = detail::SecPerm::Disabled,
    detail::SecPerm write = detail::SecPerm::Disabled,
    bool writeNoResp = false,
    bool notify = false,
    bool indicate = false,
    bool broadcast = false
>
struct Permissions : detail::PermissionsImpl<read, write, writeNoResp, notify, indicate, broadcast> {
    /// @brief Enable `read` operation (no security)
    using AllowRead = Permissions<detail::SecPerm::Unprotected, write, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `write` operation (no security)
    using AllowWrite = Permissions<read, detail::SecPerm::Unprotected, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `write without response`
    using AllowWriteNoResponse = Permissions<read, write, true, notify, indicate, broadcast>;
    /// @brief Enable `notify`
    using AllowNotify = Permissions<read, write, writeNoResp, true, indicate, broadcast>;
    /// @brief Enable `indicate`
    using AllowIndicate = Permissions<read, write, writeNoResp, notify, true, broadcast>;
    /// @brief Enable `broadcast`
    using AllowBroadcast = Permissions<read, write, writeNoResp, notify, indicate, true>;

    /// @brief Enable `read` with encryption required
    using AllowEncryptedRead = Permissions<detail::SecPerm::Encrypted, write, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `read` with authentication required
    using AllowAuthenticatedRead = Permissions<detail::SecPerm::Authenticated, write, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `read` with authorization required
    using AllowAuthorizedRead = Permissions<detail::SecPerm::Authorized, write, writeNoResp, notify, indicate, broadcast>;

    /// @brief Enable `write` with encryption required
    using AllowEncryptedWrite = Permissions<read, detail::SecPerm::Encrypted, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `write` with authentication required
    using AllowAuthenticatedWrite = Permissions<read, detail::SecPerm::Authenticated, writeNoResp, notify, indicate, broadcast>;
    /// @brief Enable `write` with authorization required
    using AllowAuthorizedWrite = Permissions<read, detail::SecPerm::Authorized, writeNoResp, notify, indicate, broadcast>;
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
    using CallbacksConfig = blex_core::extract_char_callbacks<Args...>::type;

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
                  (blex_core::CallbackTraits<T, SubscribeHandler>::is_valid_on_subscribe && (Perms::canNotify || Perms::canIndicate)),
                  "Invalid BLE OnSubscribe callback: must be invocable with 'uint16_t' and characteristic must allow Notify or Indicate");

    static_assert(StatusHandler == nullptr ||
                  blex_core::CallbackTraits<T, StatusHandler>::is_valid_on_status,
                  "Invalid BLE OnStatus callback: must be invocable with 'int'");


    // Type metadata (backend-agnostic)
    using value_type = T;
    static constexpr auto uuid = UUID;
    using perms_type = Perms;
    using descriptors_pack = blex_core::filter_descriptors_from_args<Args...>::type;
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

    /// Characteristics pack - public typedef for SFINAE detection in multiple inheritance
    /// This ensures chars_pack is unambiguous even when Service is one of multiple bases
    using chars_pack = blex_core::CharsPack<Chars...>;

    // Validate all characteristics' descriptors at compile-time
    static constexpr void validate() {
        (Chars::validate_all_descriptors(), ...);
    }
};

// ---------------------- Manufacturer Data Builder ----------------------

namespace blex_core {

// TLV (Type-Length-Value) constants for manufacturer data
namespace TLV {
    constexpr uint8_t DEVICE_TYPE = 0x01;      ///< Device type identifier
    // Add more custom TLV types as needed
}

/**
 * @brief TLV entry for manufacturer data
 * @tparam Type TLV type identifier
 * @tparam Values TLV value bytes
 */
template<uint8_t Type, uint8_t... Values>
struct TLVEntry {
    static constexpr uint8_t data[] = {Type, sizeof...(Values), Values...};
    static constexpr size_t size = sizeof(data);
};

template<uint8_t Type, uint8_t... Values>
constexpr uint8_t TLVEntry<Type, Values...>::data[];

/**
 * @brief Manufacturer data builder using TLV format
 * @details Apple-compatible TLV (Type-Length-Value) format
 *
 * # Format:
 * - Bytes 0-1: Manufacturer ID (uint16_t, little-endian)
 * - Byte 2+:   TLV entries (Type, Length, Value...)
 *
 * # Example:
 * @code
 * ::WithManufacturerData<>::WithManufacturerId<0x1234>
 *       ::WithDeviceType<0x01>
 *       ::WithTLV<0x02, 1, 2, 3>
 * @endcode
 */
template<uint16_t ManufacturerId = 0xFFFF, uint8_t... TLVBytes>
struct ManufacturerDataBuilder {
    static constexpr uint8_t data[] = {
        static_cast<uint8_t>(ManufacturerId & 0xFF),
        static_cast<uint8_t>((ManufacturerId >> 8) & 0xFF),
        TLVBytes...
    };

    static constexpr size_t size = sizeof(data);

    /// Set manufacturer ID
    template<uint16_t NewId>
    using WithManufacturerId = ManufacturerDataBuilder<NewId, TLVBytes...>;

    /// Add TLV entry (Type, Length auto-calculated, Value...)
    template<uint8_t Type, uint8_t... Values>
    using WithTLV = ManufacturerDataBuilder<ManufacturerId, TLVBytes..., Type, sizeof...(Values), Values...>;

    /// Add device type TLV (convenience method)
    template<uint8_t DeviceType>
    using WithDeviceType = WithTLV<TLV::DEVICE_TYPE, DeviceType>;
};

// Storage for a data array
template<uint16_t ManufacturerId, uint8_t... TLVBytes>
constexpr uint8_t ManufacturerDataBuilder<ManufacturerId, TLVBytes...>::data[];

} // namespace blex_core

// ---------------------- Configuration Templates ----------------------

// Internal implementation details for AdvertisementConfig (hidden from user-facing API)
namespace adv_config_detail {
    // Exit validation helpers (hidden from Builder's public API)
    template<
        int8_t TxPower,
        uint16_t IntervalMin,
        uint16_t IntervalMax,
        auto Appearance,
        const char* LongName,
        uint16_t ManufacturerId,
        bool ManufacturerIdSet,
        uint8_t... TLVBytes
    >
    struct BuilderExitValidation {
        static constexpr void validate_complete() noexcept {
            static_assert(ManufacturerIdSet,
                "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "❌  BLEX Manufacturer Data Builder Error\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "\n"
                "Manufacturer ID is required when using builder mode!\n"
                "\n"
                "You must call:\n"
                "  ::WithManufacturerId<...>  (required)\n"
                "\n"
                "Optional TLV entries:\n"
                "  ::WithDeviceType<...>        // Convenience for device type TLV\n"
                "  ::WithTLV<Type, Values...>   // Custom TLV entries\n"
                "\n"
                "✓ Correct (minimal):\n"
                "    ::WithManufacturerData<>\n"
                "        ::WithManufacturerId<0x1234>\n"
                "    ::WithAppearance<...>\n"
                "\n"
                "✓ Correct (with TLV):\n"
                "    ::WithManufacturerData<>\n"
                "        ::WithManufacturerId<0x1234>\n"
                "        ::WithDeviceType<0x01>\n"
                "    ::WithAppearance<...>\n"
                "\n"
                "✗ Missing ManufacturerId:\n"
                "    ::WithManufacturerData<>\n"
                "        ::WithDeviceType<0x01>  // ← Missing WithManufacturerId!\n"
                "    ::WithAppearance<...>\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        }

        template<int8_t NewTxPower>
        struct ExitWithTxPower_Impl {
            static constexpr auto _ = (validate_complete(), 0);
            using type = AdvertisementConfig<NewTxPower, IntervalMin, IntervalMax, Appearance,
                blex_core::ManufacturerDataBuilder<ManufacturerId, TLVBytes...>::data, LongName>;
        };

        template<uint16_t NewMin, uint16_t NewMax>
        struct ExitWithIntervals_Impl {
            static constexpr auto _ = (validate_complete(), 0);
            using type = AdvertisementConfig<TxPower, NewMin, NewMax, Appearance,
                blex_core::ManufacturerDataBuilder<ManufacturerId, TLVBytes...>::data, LongName>;
        };

        template<auto NewAppearance>
        struct ExitWithAppearance_Impl {
            static constexpr auto _ = (validate_complete(), 0);
            using type = AdvertisementConfig<TxPower, IntervalMin, IntervalMax, NewAppearance,
                blex_core::ManufacturerDataBuilder<ManufacturerId, TLVBytes...>::data, LongName>;
        };
    };

    /// @brief Dual-mode manufacturer data implementation (internal - do not use directly)
    template<
        int8_t TxPower,
        uint16_t IntervalMin,
        uint16_t IntervalMax,
        auto Appearance,
        const char* LongName,
        const auto&... Data
    >
    struct WithManufacturerDataT {
        static constexpr size_t param_count = sizeof...(Data);

        // Validate parameter count
        static_assert(param_count <= 1,
            "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            "❌  BLEX Manufacturer Data Error\n"
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            "\n"
            "WithManufacturerData accepts ONLY:\n"
            "  - 0 parameters (builder mode)\n"
            "  - 1 parameter (raw value - uint8_t[])\n"
            "\n"
            "✓ Correct: ::WithManufacturerData<>\n"
            "✓ Correct: ::WithManufacturerData<myData>\n"
            "✗ Invalid: ::WithManufacturerData<myData, otherData>\n"
            "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

        // Builder mode implementation (when param_count == 0)
        template<
            uint16_t ManufacturerId = 0xFFFF,
            bool ManufacturerIdSet = false,
            uint8_t... TLVBytes
        >
        struct Builder {
            using ExitHelper = BuilderExitValidation<TxPower, IntervalMin, IntervalMax, Appearance, LongName,
                                                      ManufacturerId, ManufacturerIdSet, TLVBytes...>;

            /// @brief Marker for trait detection - allows builder to be recognized as AdvertisementConfig
            using is_blex_advertisement_config_tag = void;

            /// @brief Default type resolution - allows builder to be used as final config without explicit exit
            /// @details Validates manufacturer ID is set and returns AdvertisementConfig with current state.
            ///          Note: This creates the final AdvertisementConfig directly without validation,
            ///          relying on compile-time checks at the point of actual use.
            using type = AdvertisementConfig<TxPower, IntervalMin, IntervalMax, Appearance,
                blex_core::ManufacturerDataBuilder<ManufacturerId, TLVBytes...>::data, LongName>;

            /// Set manufacturer ID
            template<uint16_t NewId>
            using WithManufacturerId = Builder<NewId, true, TLVBytes...>;

            /// Add TLV entry (Type, Length auto-calculated, Value...)
            template<uint8_t Type, uint8_t... Values>
            using WithTLV = Builder<ManufacturerId, ManufacturerIdSet, TLVBytes..., Type, sizeof...(Values), Values...>;

            /// Add device type TLV (convenience method)
            template<uint8_t DeviceType>
            using WithDeviceType = WithTLV<blex_core::TLV::DEVICE_TYPE, DeviceType>;

            // Exit methods - validate and return AdvertisementConfig (validation helpers in detail namespace)
            template<int8_t NewTxPower>
            using WithTxPower = ExitHelper::template ExitWithTxPower_Impl<NewTxPower>::type;

            template<uint16_t NewMin, uint16_t NewMax>
            using WithIntervals = ExitHelper::template ExitWithIntervals_Impl<NewMin, NewMax>::type;

            template<auto NewAppearance>
            using WithAppearance = ExitHelper::template ExitWithAppearance_Impl<NewAppearance>::type;
        };

        // Raw value mode implementation (when param_count == 1)
        template<const auto& RawData>
        struct RawValue {
            // Validate it's uint8_t[]
            using RawDataType = std::remove_reference_t<decltype(RawData)>;

            static_assert(std::is_array_v<RawDataType>,
                "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "❌  BLEX Manufacturer Data Type Error\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "\n"
                "Raw value must be uint8_t[] type!\n"
                "\n"
                "✓ Correct:\n"
                "    static constexpr uint8_t myData[] = {0x12, 0x34, ...};\n"
                "    ::WithManufacturerData<myData>\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

            using ElementType = std::remove_cv_t<std::remove_extent_t<RawDataType>>;

            static_assert(std::is_same_v<ElementType, uint8_t>,
                "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "❌  BLEX Manufacturer Data Type Error\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                "\n"
                "Array elements must be uint8_t!\n"
                "\n"
                "✓ Correct: uint8_t myData[] = {...}\n"
                "✗ Invalid: uint16_t myData[] = {...}\n"
                "✗ Invalid: int myData[] = {...}\n"
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

            // Exit methods - return AdvertisementConfig with raw data
            template<int8_t NewTxPower>
            using WithTxPower = AdvertisementConfig<NewTxPower, IntervalMin, IntervalMax, Appearance, RawData, nullptr>;

            template<uint16_t NewMin, uint16_t NewMax>
            using WithIntervals = AdvertisementConfig<TxPower, NewMin, NewMax, Appearance, RawData, nullptr>;

            template<auto NewAppearance>
            using WithAppearance = AdvertisementConfig<TxPower, IntervalMin, IntervalMax, NewAppearance, RawData, nullptr>;
        };

        // Mode selection based on parameter count
        template<size_t Count, const auto&... Args>
        struct SelectMode;

        template<const auto&... Args>
        struct SelectMode<0, Args...> {
            using type = Builder<>;
        };

        template<const auto& FirstData>
        struct SelectMode<1, FirstData> {
            using type = RawValue<FirstData>;
        };

        using type = SelectMode<param_count, Data...>::type;
    };
} // namespace adv_config_detail

/**
 * @brief Compile-time advertisement configuration with a fluent builder interface
 * @tparam TxPower Transmission power in dBm (-12 to 9)
 * @tparam IntervalMin Minimum advertising interval in ms (20 to 10240)
 * @tparam IntervalMax Maximum advertising interval in ms (20 to 10240)
 * @tparam Appearance BLE appearance value
 * @tparam ManufacturerData Reference to manufacturer data uint8_t[] (auto-sized)
 */
template<
    int8_t TxPower = 0,
    uint16_t IntervalMin = 100,
    uint16_t IntervalMax = 150,
    auto Appearance = 0,  // 0 = BleAppearance::kUnknown (unknown device type)
    const auto& ManufacturerData = blex_core::ManufacturerDataBuilder<>::data,
    const char* LongName = nullptr  ///< Long/full device name for scan response (nullptr = use short name)
>
struct AdvertisementConfig {
    // Validate manufacturer data type (must be uint8_t[])
    static_assert(std::is_array_v<std::remove_reference_t<decltype(ManufacturerData)>>,
        "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "❌  BLEX Manufacturer Data Type Error\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "\n"
        "Manufacturer data must be uint8_t[] type!\n"
        "\n"
        "✓ Correct:\n"
        "    static constexpr uint8_t myData[] = {0x12, 0x34, ...};\n"
        "    ::WithManufacturerData<myData>\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    static_assert(std::is_same_v<std::remove_cv_t<std::remove_extent_t<std::remove_reference_t<decltype(ManufacturerData)>>>, uint8_t>,
        "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "❌  BLEX Manufacturer Data Type Error\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "\n"
        "Array elements must be uint8_t!\n"
        "\n"
        "✓ Correct: uint8_t myData[] = {...}\n"
        "✗ Invalid: uint16_t myData[] = {...}\n"
        "✗ Invalid: int myData[] = {...}\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Validate Appearance type and range
    static_assert(std::is_same_v<decltype(Appearance), blex_standard::BleAppearance> ||
                  (std::is_integral_v<decltype(Appearance)> &&
                   static_cast<uint64_t>(Appearance) <= 0xFFFF),
                  "Appearance must be blex_standard::BleAppearance enum or uint16_t value [0x0000-0xFFFF]. "
                  "Example: blex_standard::BleAppearance::kGenericSensor or 0x0540");

    // Marker for trait detection
    using is_blex_advertisement_config_tag = void;

    // Compile-time configuration (user-specified via template parameters)
    static constexpr int8_t default_tx_power = TxPower;
    static constexpr uint16_t default_adv_interval_min = IntervalMin;
    static constexpr uint16_t default_adv_interval_max = IntervalMax;
    static constexpr uint16_t default_appearance = static_cast<uint16_t>(Appearance);
    static constexpr uint8_t default_flags = 0x06;  // LE General Discoverable + BR/EDR Not Supported

    // Manufacturer data (raw bytes, auto-sized)
    static constexpr const uint8_t* manufacturer_data = ManufacturerData;
    static constexpr size_t manufacturer_data_len = sizeof(ManufacturerData);

    /// Long device name (nullptr = use short name)
    static constexpr const char* long_name = LongName;

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
    static_assert(sizeof(ManufacturerData) <= 27,
                 "Manufacturer data length must be <= 27 bytes");

    // Fluent builder methods - each returns a new type with updated config
    template<int8_t NewTxPower>
    using WithTxPower = AdvertisementConfig<NewTxPower, IntervalMin, IntervalMax, Appearance, ManufacturerData, LongName>;

    template<uint16_t NewMin, uint16_t NewMax>
    using WithIntervals = AdvertisementConfig<TxPower, NewMin, NewMax, Appearance, ManufacturerData, LongName>;

    template<auto NewAppearance>
    using WithAppearance = AdvertisementConfig<TxPower, IntervalMin, IntervalMax, NewAppearance, ManufacturerData, LongName>;

    /// Set a long(full) device name for scan response (nullptr = use short name)
    template<const char* NewLongName>
    using WithLongName = AdvertisementConfig<TxPower, IntervalMin, IntervalMax, Appearance, ManufacturerData, NewLongName>;

    /**
     * @brief Dual-mode manufacturer data configuration
     * @details Supports two modes via variadic template parameter:
     *
     * **Mode 1: Builder (0 parameters) - Fluent chainable configuration**
     * @code{.cpp}
     * AdvertisementConfig<>
     *     ::WithManufacturerData<>
     *         ::WithManufacturerId<0x1234>
     *         ::WithDeviceType<0x05>  // uint8_t device type
     *     ::WithAppearance<kSensor>
     * @endcode
     *
     * **Mode 2: Raw value (1 parameter) - Direct uint8_t[] injection**
     * @code{.cpp}
     * static constexpr uint8_t myData[] = {0x12, 0x34, ...};
     * AdvertisementConfig<>
     *     ::WithManufacturerData<myData>
     *     ::WithAppearance<kSensor>
     * @endcode
     */
    template<const auto&... Data>
    using WithManufacturerData = adv_config_detail::WithManufacturerDataT<TxPower, IntervalMin, IntervalMax, Appearance, LongName, Data...>::type;
};

// Internal helpers for BLE unit conversions (hidden from public API)
namespace detail {
    /// @brief Convert milliseconds to BLE connection interval units (1.25ms per unit)
    constexpr uint16_t ms_to_interval_units(uint16_t ms) noexcept {
        // ms / 1.25 = ms * 4 / 5, with rounding to the nearest
        return static_cast<uint16_t>((static_cast<uint32_t>(ms) * 4 + 2) / 5);
    }

    /// @brief Convert milliseconds to BLE timeout units (10ms per unit)
    constexpr uint16_t ms_to_timeout_units(uint16_t ms) noexcept {
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
    using WithOnConnect = server_callbacks_detail::WithOnConnectImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;

    template<auto NewCallback>
    using WithOnDisconnect = server_callbacks_detail::WithOnDisconnectImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;

    template<auto NewCallback>
    using WithOnMTUChange = server_callbacks_detail::WithOnMTUChangeImpl<NewCallback, OnConnectCb, OnDisconnectCb, OnMTUChangeCb>::type;
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
    using WithOnRead = char_callbacks_detail::WithOnReadImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnWrite = char_callbacks_detail::WithOnWriteImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnStatus = char_callbacks_detail::WithOnStatusImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;

    template<auto NewCallback>
    using WithOnSubscribe = char_callbacks_detail::WithOnSubscribeImpl<NewCallback, OnReadCb, OnWriteCb, OnStatusCb, OnSubscribeCb>::type;
};

// ---------------------- Server Base (Backend-Agnostic) ----------------------

namespace blex_core {

/// @brief Helper to safely extract device name from AdvConfig
/// @details Uses SFINAE to avoid accessing AdvConfig::long_name when AdvConfig is void
template<typename AdvConfig, const char* ShortName, typename = void>
struct device_name_helper {
    static constexpr const char* value = ShortName;
};

template<typename AdvConfig, const char* ShortName>
struct device_name_helper<AdvConfig, ShortName, std::enable_if_t<!std::is_void_v<AdvConfig>>> {
    // Debug: Check if long_name is set
    static constexpr bool has_long_name_value = AdvConfig::long_name != nullptr;

    static constexpr const char* value =
        has_long_name_value ? AdvConfig::long_name : ShortName;
};

}  // namespace blex_core

/**
 * @brief Backend-agnostic server base type
 * @details Compile-time metadata: device name, services, advertising/connection config
 * Uses CRTP for automatic Base typedef - defaults to self-reference (void = self).
 * Contains no runtime state - all metadata is extracted from template parameters.
 */
template<
    const char* ShortName,
    typename Derived = void,
    typename... Args
>
struct ServerBase {
    using Base = std::conditional_t<std::is_void_v<Derived>, ServerBase, Derived>;  // CRTP

    // Extract configurations from variadic Args
    using AdvConfig = blex_core::extract_adv_config<Args...>::type;
    using ConnConfig = blex_core::extract_conn_config<Args...>::type;
    using SecurityConfig = blex_core::extract_security_config<Args...>::type;
    using CallbacksConfig = blex_core::extract_server_callbacks<Args...>::type;
    using ServicesTuple = blex_core::filter_non_config<Args...>::type;

    // Device identity
    static constexpr const char* short_name = ShortName;

    // Long name: from AdvertisingConfig if provided, otherwise use short name
    static constexpr const char* device_name =
        blex_core::device_name_helper<AdvConfig, ShortName>::value;

    // Extract callbacks from ServerCallbacks config
    static constexpr auto ConnectHandler = CallbacksConfig::on_connect;
    static constexpr auto DisconnectHandler = CallbacksConfig::on_disconnect;
    static constexpr auto MTUChangeHandler = CallbacksConfig::on_mtu_change;

    // Advertising service filtering (compile-time)
    using PassiveServices = blex_core::filter_services_pack<blex_core::is_passive_adv_pred, ServicesTuple>::type;
    using ActiveServices = blex_core::filter_services_pack<blex_core::is_active_adv_pred, ServicesTuple>::type;
};

// ---------------------- ServerBackend Trait (Forward Declaration) ----------------------

/**
 * @brief Server backend trait (specialized per BLE stack)
 * @details Provides runtime server operations (init, advertising, connections)
 */
template<typename ServerBaseT>
struct ServerBackend;

#endif // BLEX_CORE_HPP_