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
    /// Connection parameters for OTA speed optimization
    namespace connection {
        /// Fast connection parameters for OTA transfer (8-15ms interval)
        inline constexpr uint16_t FAST_MIN_INTERVAL_MS = 8;
        inline constexpr uint16_t FAST_MAX_INTERVAL_MS = 15;
    }  // namespace connection

    namespace detail {
        /**
         * @brief OTA service implementation using binary command dispatch
         * @tparam Blex blex<LockPolicy> type for characteristic definitions
         *
         * @details Implements exclusive OTA access - only one connection can own the OTA session.
         * The first connection to subscribe to CtrlChar notifications becomes the owner.
         * Commands and data from non-owner connections are rejected.
         */
        template<typename Blex>
        class OtaServiceImpl {
            using Perms = Blex::template Permissions<>;
            using CharCallbacks = Blex::template CharacteristicCallbacks<>;
            using guard_t = blex_sync::ScopedLock<Blex::template lock_policy, OtaServiceImpl>;

            /**
             * @brief OTA session state manager - handles ownership and connection speed
             * @details Encapsulates exclusive access logic. Uses CtrlChar::updateConnectionParams
             * for connection parameter updates (accessed via forward-declared CtrlChar).
             */
            class OtaSession {
                static constexpr uint16_t INVALID_HANDLE = 0xFFFF;

                uint16_t owner_handle = INVALID_HANDLE;
                bool in_fast_mode = false;

                [[nodiscard]] bool hasOwner() const { return owner_handle != INVALID_HANDLE; }
                [[nodiscard]] bool isOwner(const uint16_t handle) const { return owner_handle == handle; }
            public:
                /**
                 * @brief Try to acquire OTA session ownership
                 * @return true if acquired or already owned, false if owned by another
                 */
                bool tryAcquire(const uint16_t handle) {
                    guard_t guard;
                    if (!hasOwner()) {
                        owner_handle = handle;
                        BLEX_LOG_INFO("[OTA] Session is acquired by connection %u\n", handle);
                        return true;
                    }
                    if (owner_handle == handle) {
                        BLEX_LOG_WARN("[OTA] Session already locked by %u\n", owner_handle);
                        return true;  // Already owner
                    }
                    BLEX_LOG_WARN("[OTA] Rejecting acquire session request from %u; session already locked by %u\n", handle, owner_handle);
                    return false;
                }

                /**
                 * @brief Release OTA session and restore connection params
                 */
                void release(const uint16_t handle) {
                    guard_t guard;
                    if (isOwner(handle)) {
                        restoreConnection_unlocked(handle);
                        owner_handle = INVALID_HANDLE;
                        BLEX_LOG_INFO("[OTA] Session released by connection %u\n", handle);
                    }
                }

                /**
                 * @brief Switch to fast connection parameters for OTA transfer
                 * @note Disabled: macOS/iOS centrals ignore peripheral connection parameter requests.
                 *       The central controls connection intervals. On Linux, use sysfs to set intervals.
                 */
                void switchToFastConnection([[maybe_unused]] uint16_t handle) {
                    guard_t guard;
                    if (!in_fast_mode) {
                        BLEX_LOG_INFO("[OTA] Attempting to switch to fast connection (interval %u-%u ms)\n",
                            connection::FAST_MIN_INTERVAL_MS, connection::FAST_MAX_INTERVAL_MS);

                        CtrlChar::updateConnectionParams(handle,
                            connection::FAST_MIN_INTERVAL_MS, connection::FAST_MAX_INTERVAL_MS, 0, 4000);
                        in_fast_mode = true;

                    }
                }

            private:
                /// Called with lock held
                void restoreConnection_unlocked([[maybe_unused]] uint16_t handle) {
                    if (in_fast_mode) {
                        BLEX_LOG_INFO("[OTA] Restoring default connection params\n");
                        CtrlChar::restoreDefaultConnectionParams(handle);
                        in_fast_mode = false;
                    }
                }
            };

            /// Static session state
            inline static OtaSession session_{};

            /**
             * @brief Write response via Ctrl characteristic notify
             */
            static void ctrlWriter(const uint8_t* data, size_t len) {
                CtrlChar::setValue(data, len);
            }

            /**
             * @brief Send error response for rejected commands
             */
            static void sendErrorResponse(const uint8_t opcode) {
                protocol::response_header_t response;
                response.request_opcode = opcode;
                response.status = protocol::OPERATION_FAILED;
                ctrlWriter(reinterpret_cast<const uint8_t*>(&response), sizeof(response));
            }

            /**
             * @brief Control characteristic write handler with ownership check
             */
            static void onCtrlWrite(const uint8_t* data, const size_t len, NimBLEConnInfo& connInfo) {
                // Reject commands from a non-owner (tryAcquire is atomic)
                if (uint16_t handle = connInfo.getConnHandle(); !session_.tryAcquire(handle)) {
                    BLEX_LOG_WARN("[OTA] Command rejected from %u\n", handle);
                    if (len > 0) {
                        sendErrorResponse(data[0]);
                    }
                    return;
                }

                command_dispatcher::dispatch(data, len);
            }

            /**
             * @brief Control characteristic subscribe handler - manages ownership
             */
            static void onCtrlSubscribe(const uint16_t subValue, NimBLEConnInfo& connInfo) {
                uint16_t handle = connInfo.getConnHandle();

                if (subValue > 0) {
                    session_.tryAcquire(handle);
                } else {
                    session_.release(handle);
                }
            }

            /**
             * @brief Data characteristic write handler with ownership and speed management
             */
            static void onWriteData(const uint8_t* data, const size_t len, NimBLEConnInfo& connInfo) {
                uint16_t handle = connInfo.getConnHandle();

                // Atomically check/acquire ownership - rejects if owned by another
                if (!session_.tryAcquire(handle)) {
                    BLEX_LOG_DEBUG("[OTA] Data ignored from %u\n", handle);
                    return;
                }

                // Switch to fast connection on first data
                session_.switchToFastConnection(handle);

                guard_t guard;
                dfu::handle_write_data(ctrlWriter, data, len);
            }

            /**
             * @brief Command dispatcher - opcode + handler, payload auto-deduced
             */
            using command_dispatcher = blex_binary_command::Dispatcher<
                blex_binary_command::Command<protocol::CREATE_OBJECT, [](const protocol::create_object_payload_t& payload){
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
            blex_binary_command::Fallback<[](const uint8_t opcode, const blex_binary_command::DispatchError error) {
                guard_t guard;
                dfu::doDispatchError(ctrlWriter, opcode, error);
            }>
        >;

        public:
            /**
             * @brief DFU Control Characteristic
             * @details Receives commands (RX dispatch), sends responses via notification.
             * Subscribe handler manages OTA session ownership.
             *
             * Buffer type: C array sized from dfu_command_dispatcher::max_message_size
             */
            using CtrlChar = Blex::template Characteristic<
                typename command_dispatcher::buffer_type,
                uuids::DFU_CTRL,
                typename Perms::AllowWrite::AllowNotify,
                typename CharCallbacks::template WithOnWrite<onCtrlWrite>
                                      ::template WithOnSubscribe<onCtrlSubscribe>
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
    /// Safe here: header-only design with a single translation unit ensures no ODR violations.
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

