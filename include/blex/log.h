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
 * - **TRACE**: Very detailed trace information (most verbose)
 *
 * # Build Configuration
 * Control logging at compile time (pick one method):
 *
 * **Method 1: Symbolic flags (recommended)**
 * - `-DBLEX_LOG_LEVEL_TRACE` - Enable all logging (most verbose)
 * - `-DBLEX_LOG_LEVEL_DEBUG` - Enable DEBUG and above
 * - `-DBLEX_LOG_LEVEL_INFO` - Enable INFO and above (default)
 * - `-DBLEX_LOG_LEVEL_WARN` - Enable WARN and above
 * - `-DBLEX_LOG_LEVEL_ERROR` - Enable ERROR only
 * - `-DBLEX_LOG_LEVEL_NONE` or `-DBLEX_DISABLE_LOGGING` - Disable all logging
 *
 * **Method 2: Numeric level**
 * - `-DBLEX_LOG_LEVEL=5` - TRACE (most verbose)
 * - `-DBLEX_LOG_LEVEL=4` - DEBUG
 * - `-DBLEX_LOG_LEVEL=3` - INFO (default)
 * - `-DBLEX_LOG_LEVEL=2` - WARN
 * - `-DBLEX_LOG_LEVEL=1` - ERROR
 * - `-DBLEX_LOG_LEVEL=0` - NONE
 *
 * @note If multiple symbolic flags are set, the most verbose wins
 *
 * # Usage
 * @code
 * BLEX_LOG_ERROR("Failed: %s\n", error_msg);
 * BLEX_LOG_WARN("Unexpected condition\n");
 * BLEX_LOG_INFO("System initialized\n");
 * BLEX_LOG_DEBUG("Value: %d\n", value);
 * BLEX_LOG_DEBUG_BYTES("RX: ", buffer, length);
 * BLEX_LOG_TRACE("Entering function: %s\n", __func__);
 * BLEX_LOG_TRACE_BYTES("Packet: ", packet, packet_size);
 * @endcode
 *
 * @note Macros require BLEX_ prefix for consistency
 * @note All disabled levels compile to `((void)0)` - zero overhead
 */

#ifndef BLEX_LOG_H_
#define BLEX_LOG_H_

#include <Arduino.h>

// If multiple -DBLEX_LOG_LEVEL_* flags are set, the most verbose wins
#ifndef BLEX_LOG_LEVEL
  // Start with default
  #define BLEX_LOG_LEVEL 3  // INFO

  // Override with explicit flags (least to most verbose - last one wins)
  #if defined(BLEX_DISABLE_LOGGING) || defined(BLEX_LOG_LEVEL_NONE)
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 0
  #endif
  #ifdef BLEX_LOG_LEVEL_ERROR
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 1
  #endif
  #ifdef BLEX_LOG_LEVEL_WARN
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 2
  #endif
  #ifdef BLEX_LOG_LEVEL_INFO
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 3
  #endif
  #ifdef BLEX_LOG_LEVEL_DEBUG
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 4
  #endif
  #ifdef BLEX_LOG_LEVEL_TRACE
    #undef BLEX_LOG_LEVEL
    #define BLEX_LOG_LEVEL 5
  #endif
#endif

// Now define numeric constants for use in user code comparisons
// These are defined AFTER BLEX_LOG_LEVEL is set, so they don't conflict with -D flags
#define BLEX_LOG_LEVEL_NONE  0
#define BLEX_LOG_LEVEL_ERROR 1
#define BLEX_LOG_LEVEL_WARN  2
#define BLEX_LOG_LEVEL_INFO  3
#define BLEX_LOG_LEVEL_DEBUG 4
#define BLEX_LOG_LEVEL_TRACE 5

// Logging macros (compile out completely if disabled)
#if BLEX_LOG_LEVEL >= 1
  #define BLEX_LOG_ERROR(...) Serial.printf("BLEX:E " __VA_ARGS__)
#else
  #define BLEX_LOG_ERROR(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= 2
  #define BLEX_LOG_WARN(...) Serial.printf("BLEX:W " __VA_ARGS__)
#else
  #define BLEX_LOG_WARN(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= 3
  #define BLEX_LOG_INFO(...) Serial.printf("BLEX:I " __VA_ARGS__)
#else
  #define BLEX_LOG_INFO(...) ((void)0)
#endif

#if BLEX_LOG_LEVEL >= 4
  #define BLEX_LOG_DEBUG(...) Serial.printf("BLEX:D " __VA_ARGS__)
  #define BLEX_LOG_DEBUG_BYTES(prefix, data, size) do { \
    Serial.printf("BLEX:D %s", prefix); \
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

#if BLEX_LOG_LEVEL >= 5
  #define BLEX_LOG_TRACE(...) Serial.printf("BLEX:T " __VA_ARGS__)
  #define BLEX_LOG_TRACE_BYTES(prefix, data, size) do { \
    Serial.printf("BLEX:T %s", prefix); \
    for (size_t _i = 0; _i < (size) && _i < 16; ++_i) { \
      Serial.printf("%02X ", ((const uint8_t*)(data))[_i]); \
    } \
    if ((size) > 16) Serial.printf("..."); \
    Serial.printf("\n"); \
  } while(0)
#else
  #define BLEX_LOG_TRACE(...) ((void)0)
  #define BLEX_LOG_TRACE_BYTES(prefix, data, size) ((void)0)
#endif

#endif // LOG_H