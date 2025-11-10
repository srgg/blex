/**
 * @file ota.hpp
 * @brief Nordic DFU OTA Service - binary protocol for resumable firmware updates
 *
 * @details
 * Implements Nordic DFU (Device Firmware Update) binary protocol over BLE for
 * ESP32 OTA updates. Provides resume capability via NVS persistence and CRC32
 * verification for reliable field updates.
 *
 * # Service Structure
 * **UUID 0xFE59** (Nordic DFU Service)
 * - **Control Characteristic** (8ec90001-f315-4f60-9fb8-838830daea50)
 *   - WRITE: Binary DFU commands
 *   - NOTIFY: Binary status responses
 * - **Data Characteristic** (8ec90002-f315-4f60-9fb8-838830daea50)
 *   - WRITE_NO_RESPONSE: Binary firmware data stream
 *
 * # Binary Protocol (packed structs, little-endian)
 *
 * ## Request Opcodes
 * All requests start with 1-byte opcode:
 * - **0x01 CREATE_OBJECT**: `[opcode:1][type:1][size:4]` - Initialize OTA with firmware size
 * - **0x02 SET_RECEIPT_NOTIFICATION**: `[opcode:1][prn:2]` - Set progress notification interval (8-128 packets)
 * - **0x03 CALCULATE_CRC**: `[opcode:1]` - Request current offset and CRC32
 * - **0x04 EXECUTE_OBJECT**: `[opcode:1]` - Finalize, verify, commit, and reboot
 * - **0x06 SELECT_OBJECT**: `[opcode:1][type:1]` - Query partition state (offset, CRC, max_size)
 *
 * ## Response Format
 * All responses: `[0x60:1][req_opcode:1][status:1][data:N]`
 * - Status codes: 0x01=SUCCESS, 0x02=NOT_SUPPORTED, 0x03=INVALID_CMD, 0x04=INVALID_PARAM, 0x05=FAILED
 * - Data payloads:
 *   - **CALCULATE_CRC/SELECT_OBJECT**: `[offset:4][crc32:4]` (+ `[max_size:4]` for SELECT)
 *   - **Others**: No data payload
 *
 * # Implementation Details
 * - **Resume**: NVS stores offset/size every 32KB; client queries via SELECT_OBJECT
 * - **Verification**: CRC32 only (PRODUCTION: replace with ECDSA signature verification)
 * - **Packet Receipt Notification (PRN)**: Automatic CRC responses every N data packets
 * - **Watchdog**: Connection keepalive via periodic vTaskDelay() during long operations
 *
 * @note Thread-safe: OtaServiceImpl uses per-service ScopedLock
 * @warning CRC32 is NOT cryptographically secure - add signature verification for production
 * @see Nordic DFU Protocol: https://docs.nordicsemi.com/bundle/nrf5_SDK_v17.1.1/page/lib_dfu_transport.html
 */

#ifndef DEVICE_OTA_SVC_HPP_
#define DEVICE_OTA_SVC_HPP_

#include <vector>
#include <NimBLEDevice.h>
#include "blex.hpp"

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

    /// @brief OTA service implementation - characteristic definitions and callbacks
    template<typename Blex>
    class OtaServiceImpl {
        using Perms = typename Blex::template Permissions<>;
        using CharCallbacks = typename Blex::template CharacteristicCallbacks<>;
        using guard_t = blex_sync::ScopedLock<Blex::template lock_policy, OtaServiceImpl>;

        static void ctrlWriter(const uint8_t* data, size_t len) {
            CtrlChar::setValue(data, len);
        }

        static void onWriteCtrl(const std::vector<uint8_t>& data) {
            guard_t guard;
            DfuProcessor::instance()->handle_command(ctrlWriter, data.data(),data.size());
        }

        static void onWriteData(const std::vector<uint8_t>& data) {
            guard_t guard;
            DfuProcessor::instance()->handle_write_data(ctrlWriter, data.data(),data.size());
        }
    public:
        // DFU Ctrl
        static constexpr char DFU_CTRL_UUID_STR[] = "8ec90001-f315-4f60-9fb8-838830daea50";
        using CtrlChar = typename Blex::template Characteristic<
            std::vector<uint8_t>,
            DFU_CTRL_UUID_STR,
            typename Perms::AllowWrite::AllowNotify,
            typename CharCallbacks::template WithOnWrite<onWriteCtrl>
        >;

        // DFU Data
        static constexpr char DFU_DATA_UUID_STR[] = "8ec90002-f315-4f60-9fb8-838830daea50";
        using DataChar = typename Blex::template Characteristic<
            std::vector<uint8_t>,
            DFU_DATA_UUID_STR,
            typename Perms::AllowWriteNoResponse,
            typename CharCallbacks::template WithOnWrite<onWriteData>
        >;
    };
}

template<typename Blex, typename C = detail::OtaServiceImpl<Blex> >
struct OtaService : C, Blex::template Service<
    0xFE59,
    typename C::CtrlChar, typename C::DataChar
> {
};

#endif

