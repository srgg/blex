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
 * All requests start with a 1-byte opcode:
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
 * # Client Protocol Flows
 *
 * ## Fresh Firmware Update (Erase and Write from Zero)
 * 1. **CREATE_OBJECT** (type=DATA, size=firmware_size)
 *    - Server erases partition, sets expectedSize=firmware_size, offset=0
 *    - Response: [0x60][0x01][SUCCESS]
 * 2. **SET_RECEIPT_NOTIFICATION** (prn=16) [optional]
 *    - Server enables automatic CRC notifications every 16 data packets
 *    - Response: [0x60][0x02][SUCCESS]
 * 3. Write firmware via **Data Characteristic** (≤512-byte chunks, no response)
 *    - Server increments offset, persists to NVS every 32KB
 *    - If PRN enabled: auto-sends [0x60][0x03][SUCCESS][offset:4][crc32:4] every N packets
 * 4. **EXECUTE_OBJECT**
 *    - Server verifies size==expectedSize, validates CRC, sets boot partition, reboots
 *    - Response: [0x60][0x04][SUCCESS] then reboot
 *
 * ## Resume After Interruption (Continue from Persisted Offset)
 * 1. **Device boots**: NVS loads {offset, expectedSize, CRC} into OtaManager static state
 * 2. **SELECT_OBJECT** (type=DATA)
 *    - Server reads current state from NVS/memory
 *    - Response: [0x60][0x06][SUCCESS][offset:4][crc32:4][max_size:4]
 * 3. **Client decision based on CRC verification**:
 *    - **CRC match**: Resume - write remaining bytes from offset (skip CREATE_OBJECT, no erase)
 *    - **CRC mismatch**: Restart - call CREATE_OBJECT (erases partition, resets offset=0)
 * 4. Write remaining firmware via **Data Characteristic** (from offset onwards)
 *    - Server continues incrementing offset, persisting every 32KB
 * 5. **EXECUTE_OBJECT** to finalize and reboot
 *
 * # Implementation Details
 * - **State Persistence**: NVS stores {offset, expectedSize} every 32KB + on CREATE_OBJECT
 * - **Resume Mechanism**: Static initialization loads NVS state on boot; SELECT_OBJECT queries it
 * - **Erase Control**: CREATE_OBJECT erases entire partition; resume avoids CREATE to preserve data
 * - **CRC Verification**: Incremental CRC32 cached in memory, computed from flash on first SELECT
 * - **Packet Receipt Notification (PRN)**: Auto-notification every N packets (8-128, default off)
 * - **Connection Keepalive**: vTaskDelay() every 32 packets (write), 16 blocks (CRC), 64KB (erase)
 *
 * @note Thread-safe: OtaServiceImpl uses per-service ScopedLock for all operations
 * @note Resume requires NVS persistence + client CRC verification; server doesn't auto-resume
 * @warning CRC32 detects corruption but NOT tampering - add ECDSA signature verification for production
 * @see Nordic DFU Protocol: https://docs.nordicsemi.com/bundle/nrf5_SDK_v17.1.1/page/lib_dfu_transport.html
 */

#ifndef DEVICE_OTA_SVC_HPP_
#define DEVICE_OTA_SVC_HPP_

#include "services/ota_fwd.hpp"
#include "blex/binary_command.hpp"
#include "blex/platform.hpp"

namespace ota
{
    namespace detail {
        /**
         * @brief OTA service implementation using binary command dispatch
         * @tparam Blex blex<LockPolicy> type for characteristic definitions
         */
        template<typename Blex>
        class OtaServiceImpl {
            using Perms = Blex::template Permissions<>;
            using CharCallbacks = Blex::template CharacteristicCallbacks<>;
            using guard_t = blex_sync::ScopedLock<Blex::template lock_policy, OtaServiceImpl>;

            /**
             * @brief Write response via Ctrl characteristic notify
             */
            static void ctrlWriter(const uint8_t* data, size_t len) {
                CtrlChar::setValue(data, len);
            }

            /**
             * @brief Command dispatcher - opcode + handler, payload auto-deduced
             */
            using command_dispatcher = blex_binary_command::Dispatcher<
                blex_binary_command::Command<protocol::CREATE_OBJECT,      [](const protocol::create_object_payload_t& payload){
                    guard_t guard;
                    dfu::doCreateObject(ctrlWriter, payload);
            }>,
            blex_binary_command::Command<protocol::SET_RECEIPT_NOTIFY, [](const protocol::set_prn_payload_t& payload){
                guard_t guard;
                dfu::doSetPrn(ctrlWriter, payload);
            }>,
            blex_binary_command::Command<protocol::CALCULATE_CRC, [] {
                guard_t guard;
                dfu::doCalculateCrc(ctrlWriter);
            }>,
            blex_binary_command::Command<protocol::EXECUTE_OBJECT, [] {
                guard_t guard;
                dfu::doExecuteObject(ctrlWriter);
            }>,
            blex_binary_command::Command<protocol::SELECT_OBJECT, [](const protocol::select_object_payload_t& payload) {
                guard_t guard;
                dfu::doSelectObject(ctrlWriter, payload);
            }>,
            blex_binary_command::Fallback<[](uint8_t opcode, blex_binary_command::DispatchError error) {
                guard_t guard;
                dfu::doDispatchError(ctrlWriter, opcode, error);
            }>
        >;

            /**
             * @brief Data characteristic write handler - raw firmware bytes
             */
            static void onWriteData(const uint8_t* data, const size_t len) {
                guard_t guard;
                dfu::handle_write_data(ctrlWriter, data, len);
            }
        public:
            /**
             * @brief DFU Control Characteristic
             * @details Receives commands (RX dispatch), sends responses via notification
             *
             * Buffer type: C array sized from dfu_command_dispatcher::max_message_size
             */
            using CtrlChar = Blex::template Characteristic<
                typename command_dispatcher::buffer_type,
                uuids::DFU_CTRL,
                typename Perms::AllowWrite::AllowNotify,
                typename CharCallbacks::template WithOnWrite<command_dispatcher::dispatch>
            >;

            /**
             * @brief DFU Data Characteristic
             * @details Receives raw firmware data chunks (no opcode, no protocol dispatch)
             *
             * Buffer type: Fixed 512-byte C array (max BLE packet size)
             */
            using DataChar = Blex::template Characteristic<
                uint8_t[MAX_DATA_CHUNK_SIZE],
                uuids::DFU_DATA,
                typename Perms::AllowWriteNoResponse,
                typename CharCallbacks::template WithOnWrite<onWriteData>
            >;
        };
    }  // namespace detail

    /// Suppress -Wsubobject-linkage: lambdas in template parameters create anonymous namespace types.
    /// Safe here: header-only design with single translation unit ensures no ODR violations.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
    template<typename Blex, typename C = detail::OtaServiceImpl<Blex> >
    struct OtaService : C, Blex::template Service<
        0xFE59,
        typename C::CtrlChar, typename C::DataChar
    > {
    };
#pragma GCC diagnostic pop
}  // namespace ota

#endif

