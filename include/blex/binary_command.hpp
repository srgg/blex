/**
 * @file binary_command.hpp
 * @brief Binary command dispatch - opcode-based typed command handling
 *
 * @details
 * Receives raw bytes (opcode + payload), validates size, reinterprets as typed
 * payload, and dispatches to handlers. Designed for embedded systems: static
 * buffers, no heap, minimal binary footprint via fold-based dispatch.
 *
 * Design Principles:
 *  - Zero heap allocation: C arrays sized at compile time
 *  - Compile-time validation: Concepts + static_assert catch errors early
 *  - Minimal binary footprint: Fold-based dispatch, fully inlined handlers
 *  - Type-safe dispatch: Handlers receive typed payloads, not raw bytes
 *
 * @note Header-only for force-inline to work without LTO
 * @see core.hpp for blex design patterns
 * @see services/ota.hpp for usage example
 */

#ifndef BLEX_BINARY_COMMAND_HPP_
#define BLEX_BINARY_COMMAND_HPP_

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>

#include "log.h"

namespace blex_binary_command {

namespace detail {

/**
 * @brief Message trait - validation and metadata
 * @details Everything for message validation lives here. Zero binary footprint.
 *          Internal helpers are private to prevent direct use.
 */
struct message_trait {
private:
    /// @brief Helper to check alignment (avoids alignof(void))
    template<typename, typename = void>
    struct has_one_byte_alignment : std::false_type {};

    template<typename T>
    struct has_one_byte_alignment<T, std::enable_if_t<!std::is_void_v<T>>>
        : std::bool_constant<alignof(T) == 1> {};

    /// @brief Bool trait - payload wire-safety check (SFINAE, no errors)
    template<typename Payload>
    static constexpr bool is_wire_safe_payload_v =
        std::is_void_v<Payload> ||                  // void = no payload (opcode-only)
        (std::is_trivially_copyable_v<Payload> &&   // ensure it is POD-like raw memory safe
         std::is_standard_layout_v<Payload> &&      // predictable wire format
         has_one_byte_alignment<Payload>::value);       // no padding (#pragma pack(1))

    /// @brief Helper to get sizeof(T) without triggering sizeof(void) warning
    template<typename T, typename = void>
    struct sizeof_or_zero : std::integral_constant<size_t, sizeof(T)> {};

    template<typename T>
    struct sizeof_or_zero<T, std::enable_if_t<std::is_void_v<T>>> : std::integral_constant<size_t, 0> {};

    /// @brief Bool trait - has a message interface (SFINAE, no errors)
    template<typename, typename = void>
    struct has_message_like_interface : std::false_type {};

    template<typename T>
    struct has_message_like_interface<T, std::void_t<
        decltype(T::opcode),
        typename T::payload_type,
        decltype(T::payload_size)
    >> : std::bool_constant<
        std::is_convertible_v<decltype(T::opcode), uint8_t> &&
        std::is_convertible_v<decltype(T::payload_size), size_t>
    > {};

    /**
     * @brief Base message implementation - all validation happens here
     * @tparam Opcode Message opcode (0-255)
     * @tparam Payload Payload type (must be wire-safe: trivially copyable, standard layout, alignment 1)
     */
    template<auto Opcode, typename Payload>
    struct message_like {
        static_assert(Opcode <= 255,
            "Opcode must be in range 0-255");

        static_assert(std::is_void_v<Payload> ||              // void = no payload
                      std::is_trivially_copyable_v<Payload>,  // safe memcpy
            "Payload must be trivially copyable for safe memcpy");

        static_assert(std::is_void_v<Payload> ||              // void = no payload
                      std::is_standard_layout_v<Payload>,     // predictable wire format
            "Payload must be standard layout for predictable wire format");

        static_assert(is_wire_safe_payload_v<Payload>,
            "Payload must have alignment 1 (use #pragma pack(1)) to avoid wire padding");

        static constexpr uint8_t opcode = static_cast<uint8_t>(Opcode);
        using payload_type = Payload;
        static constexpr size_t payload_size = sizeof_or_zero<Payload>::value;
    };

public:
    /// @brief Opcode wrapper for as_message_like partial specialization
    template<uint8_t>
    struct opcode {};

    /// @brief Primary template
    template<typename...>
    struct as_message_like;

    /// @brief Specialization for existing message-like type
    template<typename T>
    struct as_message_like<T> : message_like<T::opcode, typename T::payload_type> {
        static_assert(has_message_like_interface<T>::value,
            "Type must have static opcode, payload_type, and payload_size members");
    };

    /// @brief Specialization for opcode<N> + Payload
    template<uint8_t Opcode, typename Payload>
    struct as_message_like<opcode<Opcode>, Payload> : message_like<Opcode, Payload> {};

    /// @brief Check if type has message-like interface
    template<typename T>
    static constexpr bool has_interface_v = has_message_like_interface<T>::value;
};

/**
 * @brief Handler trait - extracts payload type from handler signatures
 * @details Supports function pointers, lambdas, and functors (including noexcept variants).
 *          Handles both fixed-size void(const Payload&) and variable-size void(const Payload&, size_t).
 */
struct handler_trait {
private:
    template<typename T, typename = void>
    struct extract;

    /// @brief No-arg handler → void payload (partial specialization via SFINAE)
    template<typename T>
    struct extract<T, std::enable_if_t<
        std::is_same_v<T, void(*)()> || std::is_same_v<T, void(*)() noexcept>
    >> {
        using type = void;
    };

    /// @brief Functor/lambda - delegate to operator()
    template<typename T>
    struct extract<T, std::void_t<decltype(&T::operator())>>
        : extract<decltype(&T::operator())> {};

    template<typename Payload>
    struct extract<void(*)(const Payload&), void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename Payload>
    struct extract<void(*)(const Payload&) noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename Payload>
    struct extract<void(*)(Payload), std::enable_if_t<
        !std::is_same_v<Payload, void> && !std::is_reference_v<Payload>
    >> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename Payload>
    struct extract<void(*)(Payload) noexcept, std::enable_if_t<
        !std::is_same_v<Payload, void> && !std::is_reference_v<Payload>
    >> {
        using type = std::remove_cvref_t<Payload>;
    };

    // Variable-size handlers: void(const Payload&, size_t)
    template<typename Payload>
    struct extract<void(*)(const Payload&, size_t), void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename Payload>
    struct extract<void(*)(const Payload&, size_t) noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    /// @brief No-arg member function pointers (for no-arg lambdas/functors)
    template<typename T>
    struct extract<void(T::*)(), void> {
        using type = void;
    };

    template<typename T>
    struct extract<void(T::*)() noexcept, void> {
        using type = void;
    };

    template<typename T>
    struct extract<void(T::*)() const, void> {
        using type = void;
    };

    template<typename T>
    struct extract<void(T::*)() const noexcept, void> {
        using type = void;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&), void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&) noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&) const, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&) const noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(Payload), void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(Payload) noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(Payload) const, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(Payload) const noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    // Variable-size member function pointers: void(const Payload&, size_t)
    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&, size_t), void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&, size_t) noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&, size_t) const, void> {
        using type = std::remove_cvref_t<Payload>;
    };

    template<typename T, typename Payload>
    struct extract<void(T::*)(const Payload&, size_t) const noexcept, void> {
        using type = std::remove_cvref_t<Payload>;
    };

public:
    /// @brief Extract payload type from handler (function/lambda/functor)
    template<auto Handler>
    // ReSharper disable once CppRedundantTypenameKeyword
    using payload_type = typename extract<decltype(Handler)>::type;
};

/**
 * @brief Error codes for dispatch failures
 */
enum class DispatchError : uint8_t {
    none = 0,           ///< Success (for custom validators)
    unknown_opcode,     ///< No command registered for this opcode
    payload_too_small,  ///< Payload shorter than required
    payload_too_big,    ///< Payload longer than required (strict mode)
    invalid_message,    ///< Message is null or empty (no opcode)
    invalid_payload     ///< Custom validator rejection (size semantically wrong)
};

}  // namespace detail

/**
 * @brief Payload size validation policies
 * @details Policies control how payload size is validated before handler invocation.
 *          Each policy provides a static validate() member function.
 */
namespace payload {

/**
 * @brief Exact size policy - validates payload length matches expected size
 * @tparam N Expected size in bytes. N=0 defers validation to dispatcher (infers from payload type)
 *
 * @details
 * - SizeExact<> (N=0): Dispatcher validates len == sizeof(handler's payload type)
 * - SizeExact<N> (N>0): Policy validates len == N
 */
template<size_t N = 0>
struct SizeExact {
    /// @brief Validates payload length equals N (or defers if N=0)
    static constexpr detail::DispatchError validate(const uint8_t*, size_t len) noexcept {
        if constexpr (N == 0) {
            return detail::DispatchError::none;  // Validated by dispatcher
        } else {
            if (len < N) return detail::DispatchError::payload_too_small;
            if (len > N) return detail::DispatchError::payload_too_big;
            return detail::DispatchError::none;
        }
    }
};

/**
 * @brief Minimum size policy - validates payload length >= N
 * @tparam N Minimum required size in bytes
 */
template<size_t N>
struct SizeAtLeast {
    /// @brief Validates payload length is at least N bytes
    static constexpr detail::DispatchError validate(const uint8_t*, size_t len) noexcept {
        return len >= N ? detail::DispatchError::none : detail::DispatchError::payload_too_small;
    }
};

/**
 * @brief Maximum size policy - validates payload length <= N
 * @tparam N Maximum allowed size in bytes
 */
template<size_t N>
struct SizeAtMost {
    /// @brief Validates payload length is at most N bytes
    static constexpr detail::DispatchError validate(const uint8_t*, size_t len) noexcept {
        return len <= N ? detail::DispatchError::none : detail::DispatchError::payload_too_big;
    }
};

/**
 * @brief Range size policy - validates Min <= payload length <= Max
 * @tparam Min Minimum required size in bytes
 * @tparam Max Maximum allowed size in bytes
 */
template<size_t Min, size_t Max>
struct SizeBetween {
    static_assert(Min <= Max, "SizeBetween: Min must be <= Max");

    /// @brief Validates payload length is within [Min, Max] inclusive
    static constexpr detail::DispatchError validate(const uint8_t*, size_t len) noexcept {
        if (len < Min) return detail::DispatchError::payload_too_small;
        if (len > Max) return detail::DispatchError::payload_too_big;
        return detail::DispatchError::none;
    }
};

/**
 * @brief Unbounded size policy - accepts any payload size
 * @details Use when handler performs its own validation or any size is valid
 */
struct SizeUnbounded {
    /// @brief Always returns success - no size restrictions
    static constexpr detail::DispatchError validate(const uint8_t*, size_t) noexcept {
        return detail::DispatchError::none;
    }
};

/**
 * @brief Custom validator policy - delegates to user-provided function
 * @tparam Validator Function with signature DispatchError(const uint8_t* data, size_t len)
 *
 * @details Enables discrete size limits, runtime-dependent validation, or complex rules.
 *
 * Example:
 * @code
 * DispatchError discreteSizeValidator(const uint8_t*, size_t len) {
 *     return (len == 0 || len == 2 || len == 4)
 *         ? DispatchError::none
 *         : DispatchError::invalid_payload;
 * }
 * using DiscreteCmd = Command<0x05, handleDiscrete, payload::ValidateWith<discreteSizeValidator>>;
 * @endcode
 */
template<auto Validator>
struct ValidateWith {
    /// @brief Delegates validation to custom validator function
    static detail::DispatchError validate(const uint8_t* data, size_t len) {
        return Validator(data, len);
    }
};

}  // namespace payload

namespace detail {

/**
 * @brief Concept: Fallback handler signature
 * @tparam H Handler - function pointer, lambda, or callable with signature void(uint8_t opcode, DispatchError error)
 */
template<auto H>
concept FallbackHandler =
    std::is_same_v<decltype(H), void(*)(uint8_t, DispatchError)> ||
    std::is_same_v<decltype(H), void(*)(uint8_t, DispatchError) noexcept> ||
    std::is_invocable_r_v<void, decltype(H), uint8_t, DispatchError>;

/**
 * @brief Fallback handler for dispatch errors
 * @tparam Handler Function pointer, lambda, or callable with signature void(uint8_t opcode, DispatchError error)
 *
 * @details Add anywhere in Dispatcher args to handle:
 * - unknown_opcode: No command registered for opcode
 * - payload_too_small: Payload shorter than required
 * - payload_too_big: Payload longer than required (strict mode)
 * - invalid_message: Message is null or empty (no opcode)
 *
 * Examples:
 * @code
 * // Function pointer
 * void onError(uint8_t opcode, DispatchError error) {
 *     if (error == DispatchError::unknown_opcode) sendNotSupported(opcode);
 *     else sendInvalidParam(opcode);
 * }
 * using DfuDispatcher = Dispatcher<
 *     Command<0x01, onFoo>,
 *     Fallback<onError>
 * >;
 *
 * // Lambda (inline)
 * using DfuDispatcher2 = Dispatcher<
 *     Command<0x01, onFoo>,
 *     Fallback<[](uint8_t opcode, DispatchError error) {
 *         sendErrorResponse(opcode, error);
 *     }>
 * >;
 * @endcode
 */
template<auto Handler>
    requires FallbackHandler<Handler>
struct Fallback {
    static constexpr auto handler = Handler;
};

/**
 * @brief Dispatcher trait - utilities for command filtering, dispatch table building
 */
struct dispatcher_trait {
private:
    /// @brief Check for duplicate opcodes at compile time
    template<typename... Messages>
    static consteval bool has_unique_opcodes() {
        if constexpr (sizeof...(Messages) == 0) {
            return true;
        } else {
            constexpr uint8_t opcodes[] = {Messages::opcode...};
            for (size_t i = 0; i < sizeof...(Messages); ++i) {
                for (size_t j = i + 1; j < sizeof...(Messages); ++j) {
                    if (opcodes[i] == opcodes[j]) return false;
                }
            }
            return true;
        }
    }

    template<typename>
    struct is_fallback_impl : std::false_type {};

    template<auto H>
    struct is_fallback_impl<Fallback<H>> : std::true_type {};

    /// @brief Count Fallback types in a parameter pack
    template<typename... Ts>
    static constexpr size_t count_fallbacks_v = (is_fallback_impl<Ts>::value + ... + 0);

    /// @brief Extract Fallback handler from pack (returns nullptr if none)
    template<typename... Ts>
    static consteval auto find_fallback_handler() {
        if constexpr (sizeof...(Ts) == 0) {
            return static_cast<void(*)(uint8_t, DispatchError)>(nullptr);
        } else {
            return find_fallback_helper<Ts...>();
        }
    }

    template<typename T, typename... Rest>
    static consteval auto find_fallback_helper() {
        if constexpr (is_fallback_impl<T>::value) {
            return T::handler;
        } else {
            return find_fallback_handler<Rest...>();
        }
    }

    template<typename...>
    struct filter_impl { using type = std::tuple<>; };

    template<typename T, typename... Rest>
    struct filter_impl<T, Rest...> {
        // ReSharper disable once CppRedundantTypenameKeyword
        using rest = typename filter_impl<Rest...>::type;
        using type = std::conditional_t<
            is_fallback_impl<T>::value,
            rest,
            decltype(std::tuple_cat(std::declval<std::tuple<T>>(), std::declval<rest>()))
        >;
    };

    template<typename Tuple>
    struct from_tuple;

    template<typename... Commands>
    struct from_tuple<std::tuple<Commands...>> {
        using type = std::tuple<Commands...>;
        static constexpr bool unique_opcodes = has_unique_opcodes<Commands...>();
        static constexpr size_t max_payload = []() consteval {
            if constexpr (sizeof...(Commands) == 0) {
                return size_t{0};
            } else {
                size_t m = 0;
                ((m = Commands::payload_size > m ? Commands::payload_size : m), ...);
                return m;
            }
        }();
    };

public:
    /// @brief Check if the type is a valid Dispatcher argument (Command or Fallback)
    template<typename T>
    static constexpr bool is_valid_arg_v =
        is_fallback_impl<T>::value || message_trait::has_interface_v<T>;

    /// @brief Count Fallback types in pack
    template<typename... Ts>
    static constexpr size_t fallback_count_v = count_fallbacks_v<Ts...>;

    /// @brief Get Fallback handler from pack (nullptr if none)
    template<typename... Ts>
    static constexpr auto fallback_handler_v = find_fallback_handler<Ts...>();

    /// @brief Filter out Fallback from commands
    template<typename... Ts>
    // ReSharper disable once CppRedundantTypenameKeyword
    using filter = typename filter_impl<Ts...>::type;

    /// @brief Build filtered dispatch info from Args
    template<typename... Args>
    using filtered = from_tuple<filter<Args...>>;
};

}  // namespace detail

// Re-export to public namespace
using DispatchError = detail::DispatchError;
using detail::Fallback;

/**
 * @brief Command with configurable size policy - opcode + handler + validation
 * @tparam Opcode Message opcode (0-255)
 * @tparam Handler Function pointer or lambda - payload type extracted automatically
 * @tparam SizePolicy Validation policy (default: SizeExact<> for backward compatibility)
 *
 * @details Combines opcode, payload type, handler, and size policy in a single type.
 *
 * Handler signatures:
 * - Fixed-size (SizeExact<>): void(const Payload&) - length is compile-time known
 * - Variable-size (all others): void(const Payload&, size_t len) - receives actual length
 *
 * Examples:
 * @code
 * // Fixed-size (backward compatible)
 * void onSetMode(const SetModePayload& p);
 * using SetModeCmd = Command<0x01, onSetMode>;
 *
 * // Variable-size with range
 * void onConfig(const ConfigPayload& p, size_t len);
 * using ConfigCmd = Command<0x02, onConfig, payload::SizeBetween<1, 16>>;
 *
 * // Custom validator
 * DispatchError validateDiscrete(const uint8_t*, size_t len) {
 *     return (len == 0 || len == 2 || len == 4)
 *         ? DispatchError::none : DispatchError::invalid_payload;
 * }
 * using DiscreteCmd = Command<0x03, onDiscrete, payload::ValidateWith<validateDiscrete>>;
 * @endcode
 */
template<uint8_t Opcode, auto Handler, typename SizePolicy = payload::SizeExact<>>
struct Command : detail::message_trait::as_message_like<
    detail::message_trait::opcode<Opcode>, detail::handler_trait::payload_type<Handler>> {
    static constexpr auto handler = Handler;
    using size_policy = SizePolicy;
};

/**
 * @brief Command dispatcher - commands define everything
 * @tparam Args Variadic list of Command<Opcode, Handler> with optional Fallback<Handler> for error handling
 *
 * @details All-in-one handler: opcode, payload, and handler defined together.
 *
 * Example:
 * @code
 * using DfuDispatcher = Dispatcher<
 *     Command<0x01, onCreateObject>,
 *     Command<0x02, onSetPrn>,
 *     Command<0x03, onCalculateCrc>
 * >;
 * DfuDispatcher::dispatch(data, len);
 * @endcode
 */
template<typename... Args>
struct Dispatcher {
private:
    static constexpr bool has_fallback = detail::dispatcher_trait::fallback_count_v<Args...> > 0;
    static constexpr auto fallback = detail::dispatcher_trait::fallback_handler_v<Args...>;
    using filtered = detail::dispatcher_trait::filtered<Args...>;

    static_assert((detail::dispatcher_trait::is_valid_arg_v<Args> && ...),
        "blex_binary_command::Dispatcher arguments must be Command<Opcode, Handler> "
        "or Fallback<Handler>");

    static_assert(std::tuple_size_v<typename filtered::type> > 0,
        "blex_binary_command::Dispatcher requires at least one Command");

    static_assert(detail::dispatcher_trait::fallback_count_v<Args...> <= 1,
        "blex_binary_command::Dispatcher can have at most one Fallback");

    static_assert(filtered::unique_opcodes,
        "blex_binary_command::Dispatcher contains duplicate opcodes");

public:
    /// Number of opcodes handled (excluding fallback)
    static constexpr size_t opcode_count = std::tuple_size_v<typename filtered::type>;

    /// Maximum payload size across all commands
    static constexpr size_t max_payload_size = filtered::max_payload;

    /// Maximum message size (1 byte opcode + max payload)
    static constexpr size_t max_message_size = 1 + max_payload_size;

    /// C array buffer type for characteristics
    using buffer_type = uint8_t[max_message_size];

private:
    /// @brief Check if command uses fixed-size policy (SizeExact<> with N=0)
    template<typename Cmd>
    static constexpr bool is_fixed_size_policy_v =
        std::is_same_v<typename Cmd::size_policy, payload::SizeExact<>>;

    /// @brief Try to dispatch to a single command, return true if matched
    template<typename B>
    [[gnu::always_inline]] static bool try_dispatch(uint8_t opcode, const uint8_t* payload, size_t payload_len) {
        if (opcode != B::opcode) return false;

        // Size validation has two paths:
        // - SizeExact<> (N=0, default): Dispatcher validates against sizeof(payload_type).
        //   Policy's validate() would just return none, so we skip calling it entirely.
        // - All other policies (SizeExact<N>, SizeAtLeast, SizeBetween, ValidateWith, etc.):
        //   Call the policy's validate() method which performs the actual check.
        if constexpr (is_fixed_size_policy_v<B>) {
            if (payload_len < B::payload_size) {
                if constexpr (has_fallback) {
                    fallback(opcode, DispatchError::payload_too_small);
                } else {
                    BLEX_LOG_ERROR("Dispatcher: payload too small for opcode 0x%02X "
                                  "(got %u, need %u)\n",
                                  opcode, static_cast<unsigned>(payload_len),
                                  static_cast<unsigned>(B::payload_size));
                }
                return true;
            }

            if (payload_len > B::payload_size) {
                if constexpr (has_fallback) {
                    fallback(opcode, DispatchError::payload_too_big);
                } else {
                    BLEX_LOG_ERROR("Dispatcher: payload too big for opcode 0x%02X "
                                 "(got %u, expected %u)\n",
                                 opcode, static_cast<unsigned>(payload_len),
                                 static_cast<unsigned>(B::payload_size));
                }
                return true;
            }
        } else {
            const DispatchError policy_err = B::size_policy::validate(payload, payload_len);
            if (policy_err != DispatchError::none) {
                if constexpr (has_fallback) {
                    fallback(opcode, policy_err);
                } else {
                    BLEX_LOG_ERROR("Dispatcher: size policy rejected opcode 0x%02X "
                                  "(error %u, len %u)\n",
                                  opcode, static_cast<unsigned>(policy_err),
                                  static_cast<unsigned>(payload_len));
                }
                return true;
            }
        }

        // Invoke handler with appropriate signature
        if constexpr (B::payload_size == 0) {
            // No payload - void handler
            B::handler();
        } else if constexpr (is_fixed_size_policy_v<B>) {
            // Fixed-size policy: void(const Payload&)
            B::handler(*reinterpret_cast<const B::payload_type*>(payload));
        } else {
            // Variable-size policy: void(const Payload&, size_t len)
            B::handler(*reinterpret_cast<const B::payload_type*>(payload), payload_len);
        }
        return true;
    }

    /// @brief Handle unknown opcode (last in a fold chain)
    [[gnu::always_inline]] static bool handle_unknown(uint8_t opcode) {
        if constexpr (has_fallback) {
            fallback(opcode, DispatchError::unknown_opcode);
        } else {
            BLEX_LOG_ERROR("Dispatcher: unhandled opcode 0x%02X\n", opcode);
        }
        return true;
    }

    /// @brief Fold dispatch over filtered commands + unknown handler
    template<typename... Bs>
    [[gnu::always_inline]] static void dispatch_fold(const uint8_t opcode, const uint8_t* payload, const size_t payload_len,
                              std::tuple<Bs...>*) {
        (void)(try_dispatch<Bs>(opcode, payload, payload_len) || ... || handle_unknown(opcode));
    }

public:
    /**
     * @brief Fold-based dispatch - no function pointers, no runtime loop
     * @details Uses fold expression for a compile-time dispatch chain.
     *          Handlers can be inlined, no lambda overhead.
     */
    [[gnu::always_inline]] static void dispatch(const uint8_t* data, const size_t size) {
        if (data == nullptr || size == 0) {
            if constexpr (has_fallback) {
                fallback(0, DispatchError::invalid_message);
            } else {
                BLEX_LOG_ERROR("Dispatcher: null or empty message\n");
            }
            return;
        }

        const uint8_t opcode = data[0];
        const uint8_t* payload = data + 1;
        const size_t payload_len = size - 1;

        dispatch_fold(opcode, payload, payload_len,
                      static_cast<filtered::type*>(nullptr));
    }
};

}  // namespace blex_binary_command

#endif  // BLEX_BINARY_COMMAND_HPP_