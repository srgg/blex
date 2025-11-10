/**
 * @file log.h
 * @brief Zero-cost logging - compile-time eliminated in production builds
 *
 * @details
 * Lightweight logging macros that compile to no-ops when disabled, ensuring zero
 * code size and runtime overhead in production. Designed for embedded systems
 * where code space and performance are critical.
 *
 * # Log Levels
 * - **ERROR**: Critical failures requiring immediate attention
 * - **WARN**: Warnings about unexpected but recoverable conditions
 * - **INFO**: General informational messages (default)
 * - **DEBUG**: Detailed diagnostic information
 *
 * # Build Configuration
 * Control logging at compile time:
 * - `-DBLEX_DISABLE_LOGGING` - Disable all logging (production builds)
 * - `-DBLEX_LOG_LEVEL=N` - Set minimum log level (0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG)
 *
 * # Usage
 * @code
 * BLEX_LOG_INFO("System initialized\n");
 * BLEX_LOG_ERROR("Failed: %s\n", error_msg);
 * BLEX_LOG_DEBUG("Value: %d\n", value);
 * BLEX_LOG_DEBUG_BYTES("RX: ", buffer, length);
 * @endcode
 *
 * @note Macros require BLEX_ prefix for consistency
 * @note All disabled levels compile to `((void)0)` - zero overhead
 */

#ifndef BLEX_LOG_H_
#define BLEX_LOG_H_

#include <Arduino.h>

// Log levels
#define BLEX_LOG_LEVEL_NONE  0
#define BLEX_LOG_LEVEL_ERROR 1
#define BLEX_LOG_LEVEL_WARN  2
#define BLEX_LOG_LEVEL_INFO  3
#define BLEX_LOG_LEVEL_DEBUG 4

// Default log level (can be overridden by build flags)
#ifndef BLEX_LOG_LEVEL
  #ifdef BLEX_DISABLE_LOGGING
    #define BLEX_LOG_LEVEL BLEX_LOG_LEVEL_NONE
  #else
    #define BLEX_LOG_LEVEL BLEX_LOG_LEVEL_INFO  // Default: INFO and above
  #endif
#endif

// Logging macros (compile out completely if disabled)
#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_ERROR
  #define BLEX_LOG_ERROR(...) Serial.printf("❌ " __VA_ARGS__)
#else
  #define BLEX_LOG_ERROR(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_WARN
  #define BLEX_LOG_WARN(...) Serial.printf("⚠️  " __VA_ARGS__)
#else
  #define BLEX_LOG_WARN(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_INFO
  #define BLEX_LOG_INFO(...) Serial.printf(__VA_ARGS__)
  #define BLEX_LOG_DONE(...) Serial.printf("✅ " __VA_ARGS__)
#else
  #define BLEX_LOG_INFO(...) ((void)0)
  #define BLEX_LOG_DONE(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
  #define BLEX_LOG_DEBUG(...) Serial.printf("🔍 " __VA_ARGS__)
  #define BLEX_LOG_DEBUG_BYTES(prefix, data, size) do { \
    Serial.printf("🔍 %s", prefix); \
    for (size_t _i = 0; _i < (size) && _i < 16; ++_i) { \
      Serial.printf("%02X ", ((const uint8_t*)(data))[_i]); \
    } \
    if ((size) > 16) Serial.printf("..."); \
    Serial.printf("\n"); \
  } while(0)
#else
  #define BLEX_LOG_DEBUG(...) ((void)0)
  #define BLEX_LOG_DEBUG_BYTES(prefix, data, size) ((void)0)
#endif

#endif // LOG_H