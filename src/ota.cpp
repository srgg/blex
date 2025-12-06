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
 * All public OtaManager methods assume the caller holds the lock.
 *
 * @note Production considerations documented in implementation comments
 */

#include "services/ota_fwd.hpp"
#include "blex/log.h"

#include <Arduino.h>
#include <cinttypes>  // For PRIu32, PRIx32

#include <esp_ota_ops.h>
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_err.h"
#include <esp_crc.h>

#include <Preferences.h>

namespace ota {
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
     * - An attacker can craft firmware with a matching CRC32
     * - PRODUCTION REQUIREMENT: Add ECDSA signature verification using mbedtls
     *   before calling esp_ota_set_boot_partition()
     *
     * **Resume Limitations**
     * - esp_ota_begin() erases the partition; true resume requires:
     *   1. Client queries SELECT_OBJECT for offset/CRC before starting
     *   2. Client sends only the remaining bytes from the offset
     *   3. Server skips esp_ota_begin() when resuming (advanced)
     * - Current implementation: NVS persistence + client coordination
     *
     * **Rollback Strategy**
     * - No automatic rollback implemented
     * - Production should add: first-boot health check + automatic partition rollback
     * - Use esp_ota_mark_app_valid_cancel_rollback() after successful boot
     */
    class OtaManager {
        /// Persist NVS state every 32KB written
        static constexpr uint32_t PERSIST_INTERVAL = 32 * 1024;
        friend class board_specific::OtaManagerImpl;
    public:
        /**
         * @brief OTA error codes
         */
        enum Error : uint8_t {
            NONE = 0,
            ALREADY_IN_PROGRESS,
            NO_PARTITION,
            OTA_BEGIN_FAIL,
            OTA_READ_FAIL,
            OTA_WRITE_FAIL,
            OTA_END_FAIL,
            OTA_OUT_OF_SPACE,
            OTA_OUT_OF_RANGE,
            CRC_MISMATCH,
            SIZE_MISMATCH,
            PERSIST_FAIL,
            INVALID_STATE,
            PRN_NOTIFY,
            UNKNOWN
        };
    private:
        // NVS-persistable state snapshot for resume capability
        struct PersistableState {
            uint32_t written;
            uint32_t expectedSize;
        };

        // Full runtime state extending persistable fields with volatile session data
        struct RuntimeState : PersistableState {
            Error       last_error = NONE;
            uint32_t    lastPersistedOffset = 0;
            uint16_t    packetsSinceLastNotify = 0;
            uint16_t    prnValue = 0;
            uint32_t    cached_crc = 0;           ///< Incremental CRC cache
            bool        crc_initialized = false;  ///< Explicit flag: true when cached_crc is valid
        };

        static RuntimeState& state();
        static board_specific::OtaManagerImpl* getImpl();
        static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len);
    protected:
        /**
         * @brief Set last error and return it
         * @param e Error code to set
         * @return Same error code
         */
        static Error setError(Error e);
    public:
        /**
         * @brief Get last error code
         * @return Last error from OTA operation
         */
        static Error lastError();

        /**
         * @brief Initialize OTA session - erases partition and sets expected size
         * @param totalSize Total firmware size in bytes
         * @return true if successful, false on error (check lastError())
         */
        static bool begin(uint32_t totalSize);

        /**
         * @brief Write firmware data chunk
         * @param data Firmware bytes
         * @param len Chunk size (≤512 bytes)
         * @return Error code (NONE, PRN_NOTIFY, or error)
         */
        static Error write(const uint8_t* data, size_t len);

        /**
         * @brief Configure packet receipt notification interval
         * @param prn Number of packets between CRC notifications (8-128, 0=disable)
         */
        static void setPrn(uint16_t prn);

        /**
         * @brief Get current write offset
         * @return Bytes written to OTA partition
         */
        static uint32_t getOffset();

        /**
         * @brief Get CRC32 of written data
         * @return CRC32 checksum (computed from flash on first call if resuming)
         */
        static uint32_t getCrc();

        /**
         * @brief Finalize OTA - verify size, set boot partition, clear NVS
         * @return true if successful, false on error
         */
        static bool end();

        /**
         * @brief Trigger device reboot to apply new firmware
         */
        static void triggerDfu();

        /**
         * @brief Get OTA partition maximum size
         * @return Partition size in bytes
         */
        static uint32_t getMaxSize();
    private:
        /**
         * @brief Compute CRC32 of entire written partition data
         * @return CRC32 checksum (reads from flash in 512-byte chunks)
         */
        static uint32_t computePartitionCrc();
    };

    namespace board_specific {
        using OtaError = OtaManager::Error;

#ifdef ESP32
        class OtaManagerImpl {

            static constexpr auto NVS_NAMESPACE = "ota_mgr";
            static constexpr auto NVS_KEY_OFFSET = "offset";
            static constexpr auto NVS_KEY_SIZE   = "total_size";

            OtaManagerImpl() : otaOffset(0), otaTotalSize(0), otaPartition(nullptr) {
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
                if (!otaPartition) {
                    return OtaError::NO_PARTITION;
                }

                otaTotalSize = totalSize;
                otaOffset = 0;

                // Erase in chunks to prevent BLE connection timeout
                constexpr size_t ERASE_CHUNK = 64 * 1024;  // 64KB chunks
                const size_t total_erase = otaPartition->size;

                BLEX_LOG_INFO("[OTA] Erasing partition: %" PRIu32 " bytes in %" PRIu32 " chunks\n", static_cast<uint32_t>(total_erase), static_cast<uint32_t>((total_erase + ERASE_CHUNK - 1) / ERASE_CHUNK));

                for (size_t offset = 0; offset < total_erase; offset += ERASE_CHUNK) {
                    const size_t chunk_size = offset + ERASE_CHUNK > total_erase ? total_erase - offset : ERASE_CHUNK;

                    if (const esp_err_t rc = esp_partition_erase_range(otaPartition, offset, chunk_size); rc != ESP_OK) {
                        BLEX_LOG_ERROR("[OTA] esp_partition_erase_range failed at offset=%" PRIu32 ": %d (0x%X)\n", static_cast<uint32_t>(offset), rc, rc);
                        return OtaError::OTA_BEGIN_FAIL;
                    }

                    // Yield to BLE stack every chunk
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                BLEX_LOG_INFO("[OTA] Partition erased successfully\n");
                return OtaError::NONE;
            }

            // Persistent progress: store offset, expected size, expected CRC
            bool persistProgress(const OtaManager::PersistableState& memento) {
                if (memento.written != otaOffset || memento.expectedSize != otaTotalSize) {
                    BLEX_LOG_ERROR("State mismatch: written=%" PRIu32 " vs expected=%" PRIu32 ", expectedSize=%" PRIu32 " vs expected=%" PRIu32 "\n",
                        memento.written,
                        otaOffset,
                        memento.expectedSize,
                        otaTotalSize
                    );
                    return false;
                }

                // Preferences return bool for putUInt in Arduino wrapper
                const bool ok1 = prefs.putUInt(NVS_KEY_OFFSET, memento.written);
                const bool ok2 = prefs.putUInt(NVS_KEY_SIZE, memento.expectedSize);
                return ok1 && ok2;
            }

            void loadProgress(OtaManager::PersistableState& memento) {
                otaOffset = memento.written = prefs.getUInt(NVS_KEY_OFFSET);
                otaTotalSize = memento.expectedSize = prefs.getUInt(NVS_KEY_SIZE);
            }

            OtaError write(const uint8_t* data, const size_t len) {
                if (!otaPartition) {
                    return OtaError::NO_PARTITION;
                }

                // Prevent writing past the OTA partition
                if (otaOffset + len > otaPartition->size) {
                    return OtaError::OTA_OUT_OF_SPACE;
                }

                if (const esp_err_t rc = esp_partition_write(otaPartition, otaOffset, data, len); rc != ESP_OK) {
                    // Abort ota handle and mark not in progress
                    abort();
                    return OtaError::OTA_WRITE_FAIL;
                }

                otaOffset += len;
                return OtaError::NONE;
            }

            OtaError read(const size_t src_offset, const size_t size, uint8_t* dst) const
            {
                ensurePartition();

                if (!otaPartition) {
                    return OtaError::NO_PARTITION;
                }

                if (!dst || size == 0) {
                    return OtaError::NONE;
                }

                // Bounds check – do not read past the partition end
                if (src_offset + size > otaPartition->size) {
                    return OtaError::OTA_OUT_OF_RANGE;
                }

                if (const esp_err_t rc = esp_partition_read(otaPartition, src_offset, dst, size); rc != ESP_OK) {
                    // cannot read; return zero to indicate failure
                    return OtaError::OTA_READ_FAIL;
                }

                return OtaError::NONE;
            }

            void abort() {
                // NVS intentionally NOT cleared: client handles corruption via SELECT_OBJECT CRC verification.
                // Preserving NVS allows resume from last checkpoint if client deems data valid.

                // Clear runtime state only
                otaPartition = nullptr;
                otaOffset = 0;
                otaTotalSize = 0;
            }

            OtaError commit() {
                if (!otaPartition) {
                    BLEX_LOG_ERROR("[OTA] commit: no partition\n");
                    return OtaError::NO_PARTITION;
                }

                if (otaTotalSize != 0 && otaOffset != otaTotalSize) {
                    BLEX_LOG_ERROR("[OTA] commit: size mismatch, written=%" PRIu32 ", expected=%" PRIu32 "\n",
                                   otaOffset, otaTotalSize);
                    return OtaError::SIZE_MISMATCH;
                }

                // Set the new boot partition
                if (const esp_err_t rc = esp_ota_set_boot_partition(otaPartition); rc != ESP_OK) {
                    BLEX_LOG_ERROR("[OTA] esp_ota_set_boot_partition failed: %d (0x%X)\n", rc, rc);
                    otaPartition = nullptr;
                    return OtaError::UNKNOWN;
                }

                // Clear NVS and reset runtime state
                prefs.remove(NVS_KEY_OFFSET);
                prefs.remove(NVS_KEY_SIZE);
                otaOffset = 0;
                otaTotalSize = 0;
                otaPartition = nullptr;

                return OtaError::NONE;
            }

            // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
            // Non-static to maintain a consistent interface for platform-specific implementations
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
            uint32_t otaOffset;
            uint32_t otaTotalSize;

            mutable const esp_partition_t* otaPartition;
            Preferences prefs;
        };

#  else
#error OTAManager is not implemented on your platform
#  endif
    }  // namespace board_specific

    // OtaManager Implementation

    OtaManager::RuntimeState& OtaManager::state() {
        // Thread-safe lazy initialization (C++11). Assumes NVS available when first called (post-setup).
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

        if (setError(getImpl()->begin(totalSize)) != NONE) {
            return false;
        }

        // reset internal counters
        state.written = 0;
        state.expectedSize = totalSize;  // Store expected size for validation
        getImpl()->persistProgress(state);

        state.lastPersistedOffset = 0;
        state.packetsSinceLastNotify = 0;
        state.cached_crc = 0xFFFFFFFF;  // CRC32 initial value
        state.crc_initialized = true;   // Fresh start: initialize CRC for incremental updates
        setError(NONE);

        BLEX_LOG_INFO("[OTA] begin() - expectedSize=%" PRIu32 "\n", totalSize);
        return true;
    }

    OtaManager::Error OtaManager::write(const uint8_t* data, const size_t len) {
        RuntimeState& state = OtaManager::state();

        if (!data || len == 0) {
            return setError(NONE);
        }

        // State validation: expectedSize must be set by CREATE_OBJECT (fresh) or NVS load (resume)
        if (state.expectedSize == 0) {
            BLEX_LOG_ERROR("[OTA] Firmware data write rejected: expectedSize not initialized (missing CREATE_OBJECT or NVS state)\n");
            return setError(INVALID_STATE);
        }

        // Bounds check: prevent writing beyond expected firmware size
        if (state.written + len > state.expectedSize) {
            BLEX_LOG_ERROR("[OTA] Firmware write would exceed expectedSize: written=%" PRIu32 " + len=%zu > expected=%" PRIu32 "\n",
                           state.written, len, state.expectedSize);
            getImpl()->abort();
            return setError(OTA_OUT_OF_RANGE);
        }

#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        const auto flash_start = esp_timer_get_time();
#endif
        if (setError(getImpl()->write(data, len)) != NONE) {
            getImpl()->abort();
            return lastError();
        }
#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        const auto flash_us = esp_timer_get_time() - flash_start;
#endif

        state.written += len;

        // Update cached CRC incrementally (only if initialized)
#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        uint64_t crc_us = 0;
#endif
        if (state.crc_initialized) {
#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
            const auto crc_start = esp_timer_get_time();
#endif
            state.cached_crc = crc32_update(state.cached_crc, data, len);
#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
            crc_us = esp_timer_get_time() - crc_start;
#endif
        }

#if BLEX_LOG_LEVEL >= BLEX_LOG_LEVEL_DEBUG
        // Log timing every 32 packets (same as PRN interval)
        if ((state.packetsSinceLastNotify & 31) == 31) {
            BLEX_LOG_DEBUG("[OTA TIMING] flash=%llu us, crc=%llu us (chunk=%zu)\n", flash_us, crc_us, len);
        }
#endif

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
                // vTaskDelay(pdMS_TO_TICKS(1));
                taskYIELD();
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
        if constexpr (false) {
            for (size_t i = 0; i < len; ++i) {
                crc ^= data[i];
                for (int k = 0; k < 8; k++) {
                    crc = crc & 1 ? crc >> 1 ^ 0xEDB88320 : crc >> 1;
                }
            }
            return crc;
        } else {
            return esp_crc32_le(crc, data, len);
        }
    }

    uint32_t OtaManager::getCrc() {
        RuntimeState& state = OtaManager::state();

        // If resuming (written > 0 but CRC not calculated yet), compute from flash ONCE and cache it
        if (!state.crc_initialized && state.written > 0) {
            state.cached_crc = ~computePartitionCrc();
            state.crc_initialized = true;
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
            if (setError(getImpl()->read(offset, toRead, buf)) != NONE) {
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
            BLEX_LOG_ERROR("[OTA] Size mismatch: written=%" PRIu32 ", expected=%" PRIu32 "\n",
                state.written, state.expectedSize);
            return false;
        }

        setError(getImpl()->commit());

        if (lastError() != NONE) {
            BLEX_LOG_ERROR("[OTA] commit() failed with error=%d\n", lastError());
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
    namespace detail::dfu {

        /**
         * @brief Handle CREATE_OBJECT command - initialize OTA session with firmware size
         * @param writer Response writer callback
         * @param payload CREATE_OBJECT payload containing object type and size
         */
        void doCreateObject(const ResponseWriterFn writer, const protocol::create_object_payload_t& payload) {
            protocol::command_response_t response{};
            response.header.request_opcode = protocol::CREATE_OBJECT;
            response.header.status = protocol::SUCCESS;

            const uint32_t requested_size = payload.size;
            const uint32_t max_size = OtaManager::getMaxSize();

            if (payload.type != protocol::DATA) {
                response.header.status = protocol::INVALID_PARAMETER;
            } else if (requested_size == 0 || requested_size > max_size) {
                BLEX_LOG_ERROR("[OTA] Invalid firmware size: %" PRIu32 " (max=%" PRIu32 ")\n", requested_size, max_size);
                response.header.status = protocol::INVALID_PARAMETER;
            } else if (!OtaManager::begin(requested_size)) {
                BLEX_LOG_ERROR("[OTA] begin(size=%" PRIX32 ") failed: error %d\n",
                    requested_size,
                    OtaManager::lastError()
                );
                response.header.status = protocol::OPERATION_FAILED;
            }

            constexpr size_t responseLength = sizeof(protocol::response_header_t);
            writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

            if (response.header.status != protocol::SUCCESS) {
                BLEX_LOG_ERROR("[OTA CMD] CREATE_OBJECT execution failed: code %d\n", OtaManager::lastError());
            }
        }

        /**
         * @brief Handle SET_RECEIPT_NOTIFICATION command - configure CRC notification frequency
         * @param writer Response writer callback
         * @param payload SET_PRN payload containing packet count between notifications
         */
        void doSetPrn(const ResponseWriterFn writer, const protocol::set_prn_payload_t& payload) {
            protocol::command_response_t response{};
            response.header.request_opcode = protocol::SET_RECEIPT_NOTIFY;
            response.header.status = protocol::SUCCESS;

            const uint16_t prn = payload.value;
            constexpr uint16_t MIN_PRN = 8;    // At least CRC every 4KB (8 × 512)

            if (constexpr uint16_t MAX_PRN = 128; prn < MIN_PRN || prn > MAX_PRN) {
                BLEX_LOG_ERROR("[OTA] PRN value out of range: %u (valid: %u-%u)\n",
                               prn, MIN_PRN, MAX_PRN);
                response.header.status = protocol::INVALID_PARAMETER;
            } else {
                OtaManager::setPrn(prn);
                BLEX_LOG_INFO("[OTA] Set receipt notification after each %d chunks\n", prn);
            }

            constexpr size_t responseLength = sizeof(protocol::response_header_t);
            writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

            if (response.header.status != protocol::SUCCESS) {
                BLEX_LOG_ERROR("[OTA CMD] SET_PRN execution failed\n");
            }
        }

        /**
         * @brief Handle CALCULATE_CRC command - return current offset and CRC32
         * @param writer Response writer callback
         */
        void doCalculateCrc(const ResponseWriterFn writer) {
            protocol::command_response_t response{};
            response.header.request_opcode = protocol::CALCULATE_CRC;
            response.header.status = protocol::SUCCESS;

            response.data.crc.offset = OtaManager::getOffset();
            response.data.crc.crc32  = OtaManager::getCrc();

            constexpr size_t responseLength = sizeof(protocol::response_header_t) + sizeof(protocol::crc_response_t);
            writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

            BLEX_LOG_DEBUG("[OTA CMD]   CRC response: offset=%" PRIu32 " crc=0x%08" PRIX32 "\n",
                response.data.crc.offset,
                response.data.crc.crc32);
        }

        /**
         * @brief Handle EXECUTE_OBJECT command - finalize OTA and trigger device reboot
         * @param writer Response writer callback
         */
        void doExecuteObject(const ResponseWriterFn writer) {
            protocol::command_response_t response{};
            response.header.request_opcode = protocol::EXECUTE_OBJECT;
            response.header.status = protocol::SUCCESS;

            if (!OtaManager::end()) {
                BLEX_LOG_ERROR("[OTA] end() failed: error %d\n",
                               OtaManager::lastError()
                );
                response.header.status = protocol::OPERATION_FAILED;

                constexpr size_t responseLength = sizeof(protocol::response_header_t);
                writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

                BLEX_LOG_ERROR("[OTA CMD] EXECUTE_OBJECT execution failed: code %d\n", OtaManager::lastError());
            } else {
                const uint32_t bytes = OtaManager::getOffset();
                const uint32_t crc = OtaManager::getCrc();

                constexpr size_t responseLength = sizeof(protocol::response_header_t);
                writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

                BLEX_LOG_INFO("[OTA] Firmware update applied: %" PRIu32 " bytes, crc=0x%08" PRIX32 ". Rebooting device...\n", bytes, crc);
                delay(500);

                // Trigger reboot to apply new firmware
                OtaManager::triggerDfu();
            }
        }

        /**
         * @brief Handle SELECT_OBJECT command - query OTA session state (offset, CRC, max size)
         * @param writer Response writer callback
         * @param payload SELECT_OBJECT payload containing object type to query
         */
        void doSelectObject(const ResponseWriterFn writer, const protocol::select_object_payload_t& payload) {
            protocol::command_response_t response{};
            response.header.request_opcode = protocol::SELECT_OBJECT;
            response.header.status = protocol::SUCCESS;

            // Per Nordic DFU spec: only DATA object type is supported for firmware updates
            if (payload.type != protocol::DATA) {
                BLEX_LOG_WARN("[OTA] SELECT_OBJECT: unsupported object type %u (only DATA=0x02 supported)\n",
                             static_cast<unsigned>(payload.type));
                response.header.status = protocol::INVALID_PARAMETER;
                constexpr size_t responseLength = sizeof(protocol::response_header_t);
                writer(reinterpret_cast<const uint8_t*>(&response), responseLength);
                return;
            }

            // Return information about the currently selected object
            const uint32_t offset = OtaManager::getOffset();
            const uint32_t max_size = OtaManager::getMaxSize();

            response.data.select.offset = offset;
            response.data.select.max_size = max_size;

            // Only compute CRC if data has been written
            response.data.select.crc32 = offset > 0 ? OtaManager::getCrc() : 0;

            constexpr size_t responseLength = sizeof(protocol::response_header_t) + sizeof(protocol::select_response_t);
            writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

            BLEX_LOG_DEBUG("[OTA CMD]   SELECT response: offset=%" PRIu32 " crc=0x%08" PRIX32 " max_size=%" PRIu32 "\n",
                response.data.select.offset,
                response.data.select.crc32,
                response.data.select.max_size);
        }

        /**
         * @brief Handle dispatch errors from binary command processor
         * @param writer Response writer callback
         * @param opcode Original opcode that failed
         * @param error Dispatch error code
         */
        void doDispatchError(const ResponseWriterFn writer, const uint8_t opcode, const blex_binary_command::DispatchError error) {
            protocol::command_response_t response{};
            response.header.request_opcode = opcode;

            switch (error) {
                case blex_binary_command::DispatchError::none:
                    // Success - should not reach fallback handler
                    response.header.status = protocol::SUCCESS;
                    break;
                case blex_binary_command::DispatchError::payload_too_small:
                case blex_binary_command::DispatchError::payload_too_big:
                case blex_binary_command::DispatchError::invalid_message:
                case blex_binary_command::DispatchError::invalid_payload:
                    response.header.status = protocol::INVALID_COMMAND;
                    break;
                case blex_binary_command::DispatchError::unknown_opcode:
                    response.header.status = protocol::OPCODE_NOT_SUPPORTED;
                    break;
            }

            constexpr size_t responseLength = sizeof(protocol::response_header_t);
            writer(reinterpret_cast<const uint8_t*>(&response), responseLength);

            if (response.header.status != protocol::SUCCESS) {
                BLEX_LOG_ERROR("[OTA CMD] Dispatch error for opcode 0x%02X: %d\n", opcode, static_cast<int>(error));
            }
        }

        void handle_write_data(const ResponseWriterFn writer, const uint8_t* data, const size_t size) {
            BLEX_LOG_DEBUG_BYTES("[OTA DATA] ", data, size);

            // Ignore empty writes (BLE stack may send zero-length notifications on connection events)
            if (size == 0) {
                return;
            }

            if (constexpr size_t MAX_PACKET_SIZE = 512; size > MAX_PACKET_SIZE) {
                BLEX_LOG_ERROR("[OTA] Oversized data packet: %zu > %zu\n", size, MAX_PACKET_SIZE);
                return;
            }

            using  Error = OtaManager::Error;

            if (const Error error = OtaManager::write(data, size); error != Error::NONE) {
                if (error == Error::PRN_NOTIFY) {
                    protocol::command_response_t response{};

                    response.header.request_opcode = protocol::CALCULATE_CRC;
                    response.header.status = protocol::command_status_t::SUCCESS;

                    const uint32_t offset = OtaManager::getOffset();
                    const uint32_t crc32 = OtaManager::getCrc();

                    response.data.crc.offset = offset;
                    response.data.crc.crc32 = crc32;

                    constexpr size_t responseLength = sizeof(protocol::response_header_t) + sizeof(protocol::crc_response_t);
                    writer(reinterpret_cast<uint8_t*>(&response), responseLength);

                    BLEX_LOG_INFO("[OTA] PRN notification sent (offset=%" PRIu32 " crc=0x%08" PRIX32 ")\n", offset, crc32);
                } else {
                    BLEX_LOG_ERROR("[OTA] write() failed: error %d\n",
                        OtaManager::lastError()
                    );
                }
            }
        }
    }  // namespace detail::dfu
}  // namespace ota