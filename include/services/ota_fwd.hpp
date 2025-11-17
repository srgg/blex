/**
 * @file ota_fwd.hpp
 * @brief Forward declarations for OTA service implementation
 *
 * @details
 * This header provides the DfuProcessor class declaration without including
 * the full OTA service template. Used by ota.cpp to avoid template instantiation
 */

#ifndef DEVICE_OTA_FWD_HPP_
#define DEVICE_OTA_FWD_HPP_

#include <cstdint>
#include <cstddef>

// Forward declare UUID strings (defined in ota.cpp)
namespace ota_uuids {
    extern const char DFU_CTRL_UUID_STR[];
    extern const char DFU_DATA_UUID_STR[];
}

namespace detail {
    /// @brief DFU state machine - handles OTA protocol and ESP-IDF integration
    class DfuProcessor {
        DfuProcessor() = default;
        ~DfuProcessor() = default;
    public:
        using ResponseWriterFn = void(*)(const uint8_t*, size_t);
        void handle_command(ResponseWriterFn writer, const uint8_t* data, size_t size);
        void handle_write_data(ResponseWriterFn writer, const uint8_t* data, size_t size);

        static DfuProcessor* instance();

        DfuProcessor(const DfuProcessor&) = delete;
        DfuProcessor& operator=(const DfuProcessor&) = delete;
    };
}

#endif