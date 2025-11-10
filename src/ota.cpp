/**
 * @file ota.cpp
 * @brief OTA Update Service implementation - Nordic DFU protocol on ESP-IDF
 *
 * @details
 * Implements the OTA service defined in ota.hpp, providing:
 * - Nordic DFU protocol handler (DfuProcessor)
 * - ESP-IDF OTA partition management (OtaManager)
 * - NVS-backed progress persistence for resume capability
 * - Incremental CRC32 verification
 *
 * # Architecture
 * - **DfuProcessor**: Stateless protocol handler translating Nordic DFU commands
 * - **OtaManager**: Stateful OTA manager wrapping ESP-IDF esp_ota_* APIs
 * - **OtaManagerImpl**: Platform-specific implementation (ESP32 only)
 *
 * # Thread Safety
 * Synchronized via per-service lock in ota.hpp (OtaServiceImpl).
 * All public OtaManager methods assume caller holds lock.
 *
 * @note Production considerations documented in implementation comments
 */

#include "services/ota.hpp"
#include "blex/log.h"

#include <Arduino.h>

#include <esp_ota_ops.h>
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_err.h"

#include <Preferences.h>

namespace board_specific {
    class OtaManagerImpl;
}

/**
 * @brief OtaManager - ESP-IDF OTA partition manager with resume support
 *
 * @note Production Security Considerations
 *
 * **CRITICAL: CRC32 provides NO cryptographic security**
 * - CRC32 detects accidental corruption but NOT malicious tampering
 * - An attacker can craft firmware with matching CRC32
 * - PRODUCTION REQUIREMENT: Add ECDSA signature verification using mbedtls
 *   before calling esp_ota_set_boot_partition()
 *
 * **Resume Limitations**
 * - esp_ota_begin() erases the partition; true resume requires:
 *   1. Client queries SELECT_OBJECT for offset/CRC before starting
 *   2. Client sends only remaining bytes from offset
 *   3. Server skips esp_ota_begin() when resuming (advanced)
 * - Current implementation: NVS persistence + client coordination
 *
 * **Rollback Strategy**
 * - No automatic rollback implemented
 * - Production should add: first-boot health check + automatic partition rollback
 * - Use esp_ota_mark_app_valid_cancel_rollback() after successful boot
 */
class OtaManager {
    // Persist every 32KB
    static constexpr uint32_t PERSIST_INTERVAL = 32 * 1024;
    friend class board_specific::OtaManagerImpl;
public:
    enum Error : uint8_t {
        NONE = 0,
        ALREADY_IN_PROGRESS,
        NO_PARTITION,
        OTA_BEGIN_FAIL,
        OTA_READ_FAIL,
        OTA_WRITE_FAIL,
        OTA_END_FAIL,
        CRC_MISMATCH,
        SIZE_MISMATCH,
        PERSIST_FAIL,
        INVALID_STATE,
        PRN_NOTIFY,
        UNKNOWN
      };
private:
    // PersistableState - GoF Memento
    struct PersistableState {
        uint32_t written;
        uint32_t expectedSize;
    };

    // RuntimeState -
    struct RuntimeState : PersistableState {
        Error       last_error = NONE;
        bool        inProgress = false;
        uint32_t    lastPersistedOffset = 0;
        uint16_t    packetsSinceLastNotify = 0;
        uint16_t    prnValue = 0;
        uint32_t    cached_crc = 0xFFFFFFFF;  // Incremental CRC cache
    };

    static RuntimeState& state();
    static board_specific::OtaManagerImpl* getImpl();
    static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len);
protected:
    static Error setError(Error e);
public:
    static Error lastError();
    static bool begin(uint32_t totalSize);
    static Error write(const uint8_t* data, size_t len);

    static void setPrn(uint16_t prn);
    static uint32_t getOffset();
    static uint32_t getCrc();
    static bool end();
    static void triggerDfu();
    static uint32_t getMaxSize();
private:
    static uint32_t computePartitionCrc();
};

namespace board_specific {
    using OtaError = OtaManager::Error;

    #ifdef ESP32
    class OtaManagerImpl {

        static constexpr auto NVS_NAMESPACE = "ota_mgr";
        static constexpr auto NVS_KEY_OFFSET = "offset";
        static constexpr auto NVS_KEY_SIZE   = "expected_size";

        OtaManagerImpl() : otaHandle(0), otaPartition(nullptr) {
            prefs.begin(NVS_NAMESPACE, false);
        }

        ~OtaManagerImpl() {
          prefs.end();
        }

        void ensurePartition() const {
            if (!otaPartition) {
                otaPartition = esp_ota_get_next_update_partition(nullptr);
            }
        }

    public:
        // begin: prepares OTA partition and starts writing.
        OtaError begin(const uint32_t totalSize) {
            otaPartition = esp_ota_get_next_update_partition(nullptr);
            if (!otaPartition) return OtaError::NO_PARTITION;

            if (const esp_err_t rc = esp_ota_begin(otaPartition, totalSize, &otaHandle); rc != ESP_OK) {
                otaHandle = 0;
                otaPartition = nullptr;
                return OtaError::OTA_BEGIN_FAIL;
            }

            return OtaError::NONE;
        }

        // Persistent progress: store offset, expected size, expected CRC
        bool persistProgress(const OtaManager::PersistableState& memento) {
          // Preferences return bool for putUInt in Arduino wrapper
          const bool ok1 = prefs.putUInt(NVS_KEY_OFFSET, memento.written);
          const bool ok2 = prefs.putUInt(NVS_KEY_SIZE, memento.expectedSize);
          return ok1 && ok2;
        }

        void loadProgress(OtaManager::PersistableState& memento) {
            memento.written = prefs.getUInt(NVS_KEY_OFFSET);
            memento.expectedSize = prefs.getUInt(NVS_KEY_SIZE);
        }

        OtaError write(const uint8_t* data, const size_t len) {
            if (const esp_err_t rc = esp_ota_write(otaHandle, data, len); rc != ESP_OK) {
                // Abort ota handle and mark not in progress
                abort();
                return OtaError::OTA_WRITE_FAIL;
            }
            return OtaError::NONE;
        }

        OtaError read(const size_t src_offset, const size_t size, uint8_t* dst) const
        {
            ensurePartition();

            if (!otaPartition) {
                return OtaError::NO_PARTITION;
            }

            if (const esp_err_t rc = esp_partition_read(otaPartition, src_offset, dst, size); rc != ESP_OK) {
              // cannot read; return zero to indicate failure
              return OtaError::OTA_READ_FAIL;
            }

            return OtaError::NONE;
        }

        void abort() {
            if (otaHandle) {
                esp_ota_abort(otaHandle);
                otaHandle = 0;
            }

            otaPartition = nullptr;
        }

        OtaError commit() {
            if (!otaPartition || !otaHandle) {
                BLEX_LOG_ERROR("[DFU] commit: no partition or handle\n");
                return OtaError::NO_PARTITION;
            }

            esp_err_t rc = esp_ota_end(otaHandle);
            otaHandle = 0;

            if (rc != ESP_OK) {
                BLEX_LOG_ERROR("[DFU] esp_ota_end failed: %d (0x%X)\n", rc, rc);
                otaPartition = nullptr;
                return OtaError::OTA_END_FAIL;
            }

            rc = esp_ota_set_boot_partition(otaPartition);
            if (rc != ESP_OK) {
                BLEX_LOG_ERROR("[DFU] esp_ota_set_boot_partition failed: %d (0x%X)\n", rc, rc);
                otaPartition = nullptr;
                return OtaError::UNKNOWN;
            }

            prefs.remove(NVS_KEY_OFFSET);
            prefs.remove(NVS_KEY_SIZE);
            otaPartition = nullptr;

            return OtaError::NONE;
        }

        // ReSharper disable once CppMemberFunctionMayBeStatic
        void triggerDfu() {
          esp_restart();
        }

        uint32_t getMaxSize() const {
            ensurePartition();
            return otaPartition ? otaPartition->size : 0;
        }

        static OtaManagerImpl* instance() {
            static OtaManagerImpl impl;
            return &impl;
        }
    private:
        esp_ota_handle_t otaHandle;
        mutable const esp_partition_t* otaPartition;
        Preferences prefs;
    };

    #  else
      #error OTAManager is not implemented on your platform
    #  endif
}

// -------------------------------------------------------------
// OtaManager Implementation
// -------------------------------------------------------------

OtaManager::RuntimeState& OtaManager::state() {
    static RuntimeState state = []{
        RuntimeState s;
        getImpl()->loadProgress(s);
        s.lastPersistedOffset = s.written;
        return s;
    }();
    return state;
}

board_specific::OtaManagerImpl* OtaManager::getImpl() {
    return board_specific::OtaManagerImpl::instance();
}

OtaManager::Error OtaManager::setError(const Error e) {
    return state().last_error = e;
}

OtaManager::Error OtaManager::lastError() {
    return  state().last_error;
}

bool OtaManager::begin(const uint32_t totalSize) {
    RuntimeState& state = OtaManager::state();

    if ( setError(
        getImpl()->begin(totalSize))!= NONE  ) {
    // if (const Error error = getImpl()->begin(totalSize); error != NONE) {
    //     setError(error);
        return false;
    }

    // reset internal counters
    state.written = 0;
    state.expectedSize = totalSize;  // Store expected size for validation
    getImpl()->persistProgress(state);

    state.lastPersistedOffset = 0;
    state.packetsSinceLastNotify = 0;
    state.cached_crc = 0xFFFFFFFF;  // Reset CRC cache for fresh start
    setError(NONE);

    BLEX_LOG_INFO("[DFU] begin() - expectedSize=%u\n", totalSize);
    return true;
}

OtaManager::Error OtaManager::write(const uint8_t* data, const size_t len) {
    RuntimeState& state = OtaManager::state();

    if (!data || len == 0) {
        return setError(NONE);
    }

    if (setError(
        getImpl()->write(data, len)
    ) != NONE) {
        getImpl()->abort();
        return lastError();
    }

    state.written += len;

    // Update cached CRC incrementally (only if already initialized by getCrc())
    if (state.cached_crc != 0xFFFFFFFF) {
        state.cached_crc = crc32_update(state.cached_crc, data, len);
    }

    // persist occasionally
    if (state.written - state.lastPersistedOffset >= PERSIST_INTERVAL) {
        if (!getImpl()->persistProgress(state)) {
            // persistent failure is not fatal; just record error
            return setError(PERSIST_FAIL);
        }
        state.lastPersistedOffset = state.written;
    }

    if (state.prnValue > 0) {
        ++state.packetsSinceLastNotify;

        // Yield every 32 packets to service BLE connection events
        if ((state.packetsSinceLastNotify & 31) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (state.packetsSinceLastNotify >= state.prnValue) {
            state.packetsSinceLastNotify = 0;
            return PRN_NOTIFY;
        }
    }

    return setError(NONE);
}

void OtaManager::setPrn(const uint16_t prn) {
    state().prnValue = prn;
}

uint32_t OtaManager::getOffset() {
    return state().written;
}

uint32_t OtaManager::crc32_update(uint32_t crc, const uint8_t* data, const size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++)
            // ReSharper disable once CppRedundantParentheses
            crc = crc & 1 ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
    return crc;
}

uint32_t OtaManager::getCrc() {
    RuntimeState& state = OtaManager::state();

    // If resuming (written > 0 but CRC not calculated yet), compute from flash ONCE and cache it
    if (state.cached_crc == 0xFFFFFFFF && state.written > 0) {
        state.cached_crc = ~computePartitionCrc();
    }

    return ~state.cached_crc;
}

uint32_t OtaManager::computePartitionCrc() {

    const RuntimeState& state = OtaManager::state();
    uint32_t crc = 0xFFFFFFFF;
    constexpr size_t BUF_SIZE = 512;  // Match BLE packet size
    uint8_t buf[BUF_SIZE];  // Local: 512 bytes on stack (safe for BLE task)

    size_t remaining = state.written;
    size_t offset = 0;
    size_t blocks = 0;
    while (remaining > 0) {
        const size_t toRead = remaining > BUF_SIZE ? BUF_SIZE : remaining;
        if (setError(
            getImpl()->read(offset, toRead, buf)
            ) != NONE) {
            // cannot read; return zero to indicate failure
            return 0;
        }

        crc = crc32_update(crc, buf, toRead);
        offset += toRead;
        remaining -= toRead;

        // Yield every 16 blocks (8KB) to prevent connection timeout during large CRC calculations
        if ((++blocks & 15) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return ~crc;
}

bool OtaManager::end() {
    RuntimeState& state = OtaManager::state();

    // size check
    if (state.expectedSize != 0 && state.written != state.expectedSize) {
        // mismatch
        getImpl()->abort();
        setError(SIZE_MISMATCH);
        BLEX_LOG_ERROR("[DFU] Size mismatch: written=%u, expected=%u\n",
            state.written, state.expectedSize);
        return false;
    }

    setError(
        getImpl()->commit()
        );

    if (lastError() != NONE) {
        BLEX_LOG_ERROR("[DFU] commit() failed with error=%d\n", lastError());
        return false;
    }

    state.lastPersistedOffset = 0;
    state.packetsSinceLastNotify = 0;
    return true;
}

void OtaManager::triggerDfu() {
    getImpl()->triggerDfu();
}

uint32_t OtaManager::getMaxSize() {
    return getImpl()->getMaxSize();
}


// DFU protocol https://docs.nordicsemi.com/bundle/nrf5_SDK_v17.1.1/page/lib_dfu_transport.html
// Request handling https://docs.nordicsemi.com/bundle/nrf5_SDK_v17.1.1/page/group_sdk_nrf_dfu_req_handler.html#ga654d8446f2996253016f7c7713124094
// https://git.ri.se/jayendra.ellamathy/nrf52-sdk/-/blob/master/components/libraries/bootloader/dfu/nrf_dfu_req_handler.h
namespace protocol {
    #pragma pack(push, 1)
    enum dfu_op_code_t : uint8_t {
        OP_CODE_PROTOCOL_VERSION = 0x00,            /**< Protocol version. */
        OP_CODE_CREATE_OBJECT = 0x01,               /**< Create object. */
        OP_CODE_SET_RECEIPT_NOTIFICATION = 0x02,    /**< Set Packet Receipt Notification procedures. */
        OP_CODE_CALCULATE_CRC = 0x03,               /**< Calculate Checksum. */
        OP_CODE_EXECUTE_OBJECT = 0x04,              /**< Execute object. */
        OP_CODE_SELECT_OBJECT = 0x06,               /**< Select object. */
        OP_CODE_RESPONSE = 0x60,                    /**< Response (used by target to respond). */
    };

    /**
     * @brief DFU object types.
     */
    enum dfu_obj_type_t : uint8_t {
        DFU_OBJ_TYPE_INVALID,                   //!< Invalid object type.
        DFU_OBJ_TYPE_COMMAND,                   //!< Command object.
        DFU_OBJ_TYPE_DATA,                      //!< Data object.
    };

    /**
     * @brief Common header for all DFU requests/commands.
     */
    typedef struct {
        dfu_op_code_t opcode;
    } dfu_request_header_t;

    /**
     * @brief Parameters for the 'Create' command (Opcode 0x01).
     */
    typedef struct {
        dfu_obj_type_t  type;         // DFU_OBJ_TYPE_DATA(0x02) for Data/Firmware Image
        uint32_t        size;         // Total size of the object in bytes
    } dfu_create_object_params_t;

    /**
     * @brief Parameters for the 'Set Receipt Notification' command (Opcode 0x02).
     */
    typedef struct {
        uint16_t    value;       // Number of packets to send before a notification
    } dfu_set_prn_params_t;

    /**
     * @brief Parameters for the 'Select' command (Opcode 0x06).
     */
    typedef struct {
        dfu_obj_type_t  type;         // 0x01 for Command/Init Packet, 0x02 for Data/Firmware Image
    } dfu_select_params_t;

    /**
     * @brief Union to overlay the common header with specific command parameters.
     *        This allows parsing the incoming byte stream based on the 'opcode'.
     */

    typedef union {
        dfu_request_header_t header;
        struct {
            uint8_t opcode; // Redundant, but helps readability when accessing union
            union {
                dfu_create_object_params_t create;
                dfu_set_prn_params_t set_receipt;
                dfu_select_params_t select;
                // Execute (0x04) and Calculate Checksum (0x03) have no parameters
            } params;
        };
    } dfu_request_t;

    // Status codes used in responses
    typedef enum : uint8_t {
        DFU_STATUS_SUCCESS = 0x01,
        DFU_STATUS_OPCODE_NOT_SUPPORTED = 0x02,
        DFU_STATUS_INVALID_COMMAND = 0x03,
        DFU_STATUS_INVALID_PARAMETER = 0x04,
        DFU_STATUS_OPERATION_FAILED = 0x05
    } dfu_status_code_t;

    /**
     * @brief Common header for all DFU responses (Notifications).
     */
    typedef struct {
        uint8_t response_opcode;    // Always 0x60 (DFU_OP_RESPONSE_CODE)
        uint8_t request_opcode_id;  // The opcode of the command this is responding to
        uint8_t status;             // Status code (e.g., 0x01 for Success)
    } dfu_response_header_t;

    /**
     * @brief Data returned with 'Calculate Checksum' (Opcode 0x03) responses.
     */
    typedef struct {
        uint32_t offset;            // Current byte offset in the object
        uint32_t crc32;             // CRC32 checksum of data up to the offset
    } dfu_checksum_data_t;

    /**
     * @brief Data payload for the 'Select' command response (Opcode 0x06).
     * Extends checksum data with partition size information.
     */
    typedef struct dfu_select_response_data_t : dfu_checksum_data_t {
        uint32_t max_size;          // Maximum partition size available
    } dfu_select_response_data_t;

    /**
     * @brief Union to interpret the incoming byte stream of a notification.
     */
    typedef union {
        dfu_response_header_t header;
        struct {
            uint8_t response_opcode;
            uint8_t request_opcode_id;
            uint8_t status;
            union {
                // Checksum and Select responses share the offset/crc format
                dfu_checksum_data_t checksum_data;
                dfu_select_response_data_t select_response_data;
                // Other responses have no data payload
            } data;
        };
    } dfu_response_t;

    #pragma pack(pop)
}

namespace detail {

    DfuProcessor* DfuProcessor::instance() {
        static  DfuProcessor dfuProcessor;
        return &dfuProcessor;
    }

    // ReSharper disable once CppMemberFunctionMayBeStatic
    void DfuProcessor::handle_command(const ResponseWriterFn writer, const uint8_t* data, size_t size) {
        using namespace protocol;

        if (size == 0) {
            BLEX_LOG_WARN("[DFU] Empty command\n");
            return;
        }

        // Log all command bytes for debugging
        BLEX_LOG_DEBUG_BYTES("[DFU CMD] ", data, size);

        auto request = reinterpret_cast<const dfu_request_t*>(data);

        // Prepare response
        dfu_response_t response{};

        response.header.response_opcode = OP_CODE_RESPONSE;
        response.header.request_opcode_id = request->header.opcode;
        response.header.status = DFU_STATUS_SUCCESS; // Optimistic default
        size_t responseLength = sizeof(dfu_response_header_t);

        switch (request->header.opcode) {
            // case OP_CODE_PROTOCOL_VERSION:
            //     // Response with only a header, success status.
            //     break;

        case OP_CODE_CREATE_OBJECT:
            if (size >= sizeof(dfu_request_header_t) + sizeof(dfu_create_object_params_t)) {
                const uint32_t requested_size = request->params.create.size;
                const uint32_t max_size = OtaManager::getMaxSize();

                if (request->params.create.type != DFU_OBJ_TYPE_DATA) {
                    response.header.status = DFU_STATUS_INVALID_PARAMETER;
                } else if (requested_size == 0 || requested_size > max_size) {
                    BLEX_LOG_ERROR("[DFU] Invalid firmware size: %u (max=%u)\n", requested_size, max_size);
                    response.header.status = DFU_STATUS_INVALID_PARAMETER;
                } else if (!OtaManager::begin(requested_size)) {
                    BLEX_LOG_ERROR("[DFU] begin(size=%X) failed: error %d\n",
                        requested_size,
                        OtaManager::lastError()
                    );

                    response.header.status = DFU_STATUS_OPERATION_FAILED;
                }
            } else {
                response.header.status = DFU_STATUS_INVALID_PARAMETER;
            }
            break;

        case OP_CODE_SET_RECEIPT_NOTIFICATION:
            if (size >= sizeof(dfu_request_header_t) + sizeof(dfu_set_prn_params_t)) {
                uint16_t prn = request->params.set_receipt.value;
                constexpr uint16_t MIN_PRN = 8;    // At least CRC every 4KB (8 × 512)
                constexpr uint16_t MAX_PRN = 128;  // At most CRC every 64KB (128 × 512)

                if (prn < MIN_PRN || prn > MAX_PRN) {
                    BLEX_LOG_ERROR("[DFU] PRN value out of range: %u (valid: %u-%u)\n",
                                   prn, MIN_PRN, MAX_PRN);
                    response.header.status = DFU_STATUS_INVALID_PARAMETER;
                } else {
                    OtaManager::setPrn(prn);
                    BLEX_LOG_INFO("[DFU] Set receipt notification after each %d chunks\n", prn);
                }
            } else {
                response.header.status = DFU_STATUS_INVALID_PARAMETER;
            }
            break;

        case OP_CODE_CALCULATE_CRC:
            // Calculate the current CRC of the written data
            response.data.checksum_data.offset = OtaManager::getOffset();
            response.data.checksum_data.crc32  = OtaManager::getCrc();
            responseLength += sizeof(dfu_checksum_data_t);
            break;

        case OP_CODE_EXECUTE_OBJECT: {
            // The update is complete. Get firmware info before committing
            if (!OtaManager::end()) {
                BLEX_LOG_ERROR("[DFU] end() failed: error %d\n",
                               OtaManager::lastError()
                );
                response.header.status = DFU_STATUS_OPERATION_FAILED;
            } else {
                const uint32_t bytes = OtaManager::getOffset();
                const uint32_t crc = OtaManager::getCrc();
                writer(reinterpret_cast<const uint8_t*>(&response), responseLength);
                BLEX_LOG_DONE("[DFU] Firmware update applied: %u bytes, crc=0x%08X. Rebooting device...\n", bytes, crc);
                delay(500);

                // Trigger reboot to apply new  firmware
                OtaManager::triggerDfu();
                return;
            }
            break;
        }

        case OP_CODE_SELECT_OBJECT: {
                // Return information about the currently selected object (offset, CRC, max size)
                const uint32_t offset = OtaManager::getOffset();
                const uint32_t max_size = OtaManager::getMaxSize();

                response.data.select_response_data.offset = offset;
                response.data.select_response_data.max_size = max_size;

                // Only compute CRC if data has been written
                response.data.select_response_data.crc32 = (offset > 0) ? OtaManager::getCrc() : 0;

                responseLength += sizeof(dfu_select_response_data_t);
                break;
        }

        default:
            response.header.status = DFU_STATUS_OPCODE_NOT_SUPPORTED;
            break;
        }

        writer(reinterpret_cast<const uint8_t*>(&response), responseLength);
        if (response.header.status != DFU_STATUS_SUCCESS) {
            BLEX_LOG_ERROR("[DFU CMD] Command %d execution failed: code %d\n", request->header.opcode, OtaManager::lastError());
        } else {
            BLEX_LOG_DEBUG("[DFU CMD]   Execution response sent for opcode=0x%02X len=%d\n", request->header.opcode, responseLength);
        }
    }

    // ReSharper disable once CppMemberFunctionMayBeStatic
    void DfuProcessor::handle_write_data(const ResponseWriterFn writer, const uint8_t* data, const size_t size) {
        BLEX_LOG_DEBUG_BYTES("[DFU DATA] ", data, size);

        if (size == 0) {
            return;
        }

        constexpr size_t MAX_PACKET_SIZE = 512;  // Match BLE MTU minus overhead
        if (size > MAX_PACKET_SIZE) {
            BLEX_LOG_ERROR("[DFU] Oversized data packet: %zu > %zu\n", size, MAX_PACKET_SIZE);
            return;
        }

        using  Error = OtaManager::Error;

        if (const Error error = OtaManager::write(data, size); error != Error::NONE) {
            if (error == Error::PRN_NOTIFY) {
                protocol::dfu_response_t response{};
                response.header.response_opcode = protocol::OP_CODE_RESPONSE;
                response.header.request_opcode_id = protocol::OP_CODE_CALCULATE_CRC;
                response.header.status = protocol::DFU_STATUS_SUCCESS;

                const uint32_t offset = OtaManager::getOffset();
                const uint32_t crc32 = OtaManager::getCrc();

                response.data.checksum_data.offset = offset;
                response.data.checksum_data.crc32 = crc32;

                constexpr size_t responseLength = sizeof(protocol::dfu_checksum_data_t) + sizeof(protocol::dfu_response_header_t);
                writer(reinterpret_cast<uint8_t*>(&response), responseLength);

                BLEX_LOG_INFO("[DFU] PRN notification sent (offset=%u crc=0x%08X)\n", offset, crc32);
            } else {
                BLEX_LOG_ERROR("[DFU] write() failed: error %d\n",
                    OtaManager::lastError()
                );
            }
        }
    }
}
