/**
 * @file platform.hpp
 * @brief Platform layer - policy-based synchronization with zero-cost abstraction
 *
 * @details
 * Platform abstraction layer providing configurable lock policies and automatic
 * platform detection. Enables zero-cost synchronization: compile-time selection
 * between NoLock (single-core) and FreeRTOSLock (multi-core) with no runtime overhead.
 *
 * # Lock Policies
 * - **NoLock**: Zero-overhead no-op for single-core or externally synchronized code
 * - **FreeRTOSLock**: Per-characteristic recursive mutex for multi-core safety
 * - **DefaultLock**: Auto-selected based on platform detection
 * - **Custom**: Implement lock()/unlock() interface for alternative strategies
 *
 * # Platform Detection
 * Automatic multi-core detection for: ESP32 family, RP2040, STM32H7 dual-core
 * FreeRTOS detection across ESP-IDF, Arduino, and bare-metal configurations
 *
 * # Design Rationale
 * Policy-based design allows compile-time lock selection without vtable overhead.
 * Per-characteristic tag types ensure independent mutex instances for parallelism.
 *
 * @note Thread-safe lock initialization via C++11 function-local statics
 * @see docs/thread-safety.md for lock policy selection guide
 */

#ifndef BLEX_PLATFORM_HPP_
#define BLEX_PLATFORM_HPP_

#include <atomic>
#include <mutex>
#include <cassert>

// ---------------------- Lock Policy Implementations ----------------------

// No-op lock policy - zero overhead for single-core or externally synchronized
template<typename /*Tag*/ = void>
struct NoLock {
    void lock() const {}
    void unlock() const {}
};

// ---------------------- Platform Detection ----------------------

// Detect FreeRTOS availability across platforms
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_RP2040) || defined(STM32H7xx) || \
    defined(IDF_VER) || (defined(__has_include) && __has_include(<FreeRTOS.h>))
    #define BLEX_HAS_FREERTOS
#endif

// Detect multi-core platforms
#if defined(CONFIG_FREERTOS_UNICORE)
    // ESP32 explicitly configured as single-core
    #define BLEX_SINGLE_CORE
#elif defined(ESP32) || defined(ESP32S3) || defined(ESP32C3) || defined(ESP_PLATFORM)
    // ESP32 family (assume dual-core unless UNICORE defined)
    #define BLEX_MULTI_CORE
#elif defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040)
    // Raspberry Pi Pico (dual Cortex-M0+)
    #define BLEX_MULTI_CORE
#elif defined(STM32H745xx) || defined(STM32H747xx) || defined(STM32H755xx) || defined(STM32H757xx)
    // STM32H7 dual-core variants
    #define BLEX_MULTI_CORE
#endif

// FreeRTOS mutex implementation
#if defined(BLEX_HAS_FREERTOS) && !defined(BLEX_NO_FREERTOS)
    #if defined(ESP_PLATFORM)
        #include <freertos/FreeRTOS.h>
        #include <freertos/semphr.h>
    #else
        #include <FreeRTOS.h>
        #include <semphr.h>
    #endif

    /**
     * @brief FreeRTOS recursive mutex with ISR safety enforcement
     * @warning MUST NOT be used from an ISR context. Use only from FreeRTOS tasks.
     *          Violations will trigger assertion failure.
     */
    template<typename Tag = void>
    struct FreeRTOSLock {
        using tag = Tag;

        FreeRTOSLock() = default;
        // No destructor - never delete the static global mutex (shutdown ordering issues)

        void lock() const {
            auto m = get_mutex();
            assert(m != nullptr && "FreeRTOS mutex not initialized");
            xSemaphoreTakeRecursive(m, portMAX_DELAY);
        }

        void unlock() const {
            auto m = get_mutex();
            assert(m != nullptr && "FreeRTOS mutex not initialized");
            xSemaphoreGiveRecursive(m);
        }
    private:
        // Per-Tag mutex instance; C++11 function-local statics guarantee thread-safe lazy init
        static SemaphoreHandle_t& get_mutex() {
            #if defined(ESP_PLATFORM)
                configASSERT(!xPortInIsrContext() &&
                             "FreeRTOSLock: MUST NOT be called from ISR context!");
            #endif

            // Thread-safe init per C++11; static storage (no heap)
            static StaticSemaphore_t static_buf;
            static SemaphoreHandle_t mutex = []() -> SemaphoreHandle_t {
                auto h = xSemaphoreCreateRecursiveMutexStatic(&static_buf);
                assert(h && "FreeRTOS recursive mutex creation failed");
                return h;
            }();
            return mutex;
        }
    };
#endif

// ---------------------- Default Lock Policy Selection ----------------------

#if defined(BLEX_HAS_FREERTOS) && !defined(BLEX_NO_FREERTOS)
    // FreeRTOS available: use FreeRTOSLock for all platforms
    template<typename Tag = void>
    using DefaultLock = FreeRTOSLock<Tag>;

    #if defined(BLEX_MULTI_CORE)
        #pragma message("BLEX: Multi-core platform detected, using FreeRTOSLock for thread safety")
    #endif

#elif defined(BLEX_MULTI_CORE)
    // Multi-core platform WITHOUT FreeRTOS: ERROR
    #error "BLEX: Multi-core platform detected but FreeRTOS not available. " \
           "Either: (1) Enable FreeRTOS, or (2) Explicitly specify lock policy: blex<YourLockPolicy>"

#else
    // Single-core or unknown platform: use NoLock with a warning
    template<typename Tag = void>
    using DefaultLock = NoLock<Tag>;

    #pragma message("BLEX: No threading support detected, using NoLock (no thread safety). " \
                    "If your platform is multi-core, explicitly specify: blex<FreeRTOSLock>")
#endif

// ---------------------- Synchronization Primitives ----------------------

namespace blex_sync {

// RAII lock guard
template<typename Lock>
class LockGuard {
    Lock& lock;
public:
    explicit LockGuard(Lock& l) : lock(l) { lock.lock(); }
    ~LockGuard() { lock.unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;
};

// RAII scope lock
template<template<typename> class LockPolicy, typename Tag>
class ScopedLock {
    using Lock = LockPolicy<Tag>;

    [[gnu::always_inline]]
      static Lock& getLock() noexcept {
        // Leak-on-purpose to avoid static destruction order issues
        static Lock* lock = new Lock();
        return *lock;
    }
public:
    ScopedLock() noexcept  { getLock().lock(); }
    ~ScopedLock() noexcept { getLock().unlock(); }

    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
};

} // namespace blex_sync

#endif // BLEX_PLATFORM_HPP_