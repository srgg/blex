/**
 * @file ota_fwd.hpp
 * @brief Forward declarations for OTA service implementation
 *
 * @details
 * This header provides the DfuProcessor class declaration without including
 * the full OTA service template. Used by ota.cpp to avoid template instantiation
 */

#ifndef OTA_FWD_HPP
#define OTA_FWD_HPP

#include "blex/binary_command.hpp"

namespace ota {
    namespace uuids {
        inline constexpr char DFU_CTRL[] = "8ec90001-f315-4f60-9fb8-838830daea50";
        inline constexpr char DFU_DATA[] = "8ec90002-f315-4f60-9fb8-838830daea50";
    }

    /// DFU protocol specification: https://docs.nordicsemi.com/bundle/nrf5_SDK_v17.1.1/page/lib_dfu_transport.html
    namespace protocol {
#pragma pack(push, 1)
        enum command_t : uint8_t {
            CREATE_OBJECT = 0x01,               /**< Create object. */
            SET_RECEIPT_NOTIFY = 0x02,          /**< Set Packet Receipt Notification procedures. */
            CALCULATE_CRC = 0x03,               /**< Calculate Checksum. */
            EXECUTE_OBJECT = 0x04,              /**< Execute object. */
            SELECT_OBJECT = 0x06,               /**< Select object. */
            RESPONSE = 0x60,                    /**< Response (used by target to respond). */
        };

        enum command_status_t : uint8_t {
            SUCCESS                = 0x01,
            OPCODE_NOT_SUPPORTED   = 0x02,
            INVALID_COMMAND        = 0x03,
            INVALID_PARAMETER      = 0x04,
            OPERATION_FAILED       = 0x05,
        };

        /**
         * @brief DFU object types.
         */
        enum object_type_t : uint8_t {
            INVALID    = 0x00,          //!< Invalid object type.
            COMMAND    = 0x01,          //!< Command object.
            DATA       = 0x02,          //!< Data object.
        };

        /**
         * @brief CREATE_OBJECT payload (0x01)
         */
        struct create_object_payload_t {
            object_type_t   type;       ///< Object type (DATA = 0x02)
            uint32_t        size;       ///< Total firmware size in bytes
        };
        static_assert(sizeof(create_object_payload_t) == 5);

        /**
         * @brief SET_RECEIPT_NOTIFICATION payload (0x02)
         */
        struct set_prn_payload_t {
            uint16_t value;     ///< Packets between CRC notifications (8-128)
        };
        static_assert(sizeof(set_prn_payload_t) == 2);

        /**
         * @brief SELECT_OBJECT payload (0x06)
         */
        struct select_object_payload_t {
            object_type_t type;       ///< Object type to query
        };
        static_assert(sizeof(select_object_payload_t) == 1);

        /**
         * @brief Response header (all responses start with this)
         */
        struct response_header_t {
            const uint8_t response_opcode = RESPONSE;   ///< Always 0x60
            uint8_t request_opcode = 0;
            uint8_t status = SUCCESS;
        };
        static_assert(sizeof(response_header_t) == 3);

        /**
         * @brief CRC response data (CALCULATE_CRC, PRN notify)
         */
        struct crc_response_t {
            uint32_t offset = 0;    ///< Current byte offset
            uint32_t crc32 = 0;     ///< CRC32 of data up to offset
        };
        static_assert(sizeof(crc_response_t) == 8);

        /**
         * @brief SELECT response data (extends CRC data)
         */
        struct select_response_t : crc_response_t {
            uint32_t max_size = 0;  ///< Maximum partition size
        };
        static_assert(sizeof(select_response_t) == 12);

        /**
         * @brief Full response (header + max data)
         */
        struct command_response_t {
            response_header_t header;
            union {
                crc_response_t crc;
                select_response_t select;
            } data;
        };
        static_assert(sizeof(command_response_t) == 15);

#pragma pack(pop)
    }  // namespace protocol

    /// Maximum firmware chunk size (matches BLE MTU minus overhead)
    inline constexpr size_t MAX_DATA_CHUNK_SIZE = 512;

    namespace detail {
        /// @brief DFU protocol command handlers - stateless OTA command processors
        namespace dfu {
            /// Response writer callback - writes binary response to Control characteristic
            using ResponseWriterFn = void(*)(const uint8_t*, size_t);

            /**
             * @brief Handle firmware data write from Data characteristic
             * @param writer Response writer callback for sending notifications
             * @param data Raw firmware chunk bytes
             * @param size Chunk size in bytes (≤512)
             */
            void handle_write_data(ResponseWriterFn writer, const uint8_t* data, size_t size);

            /**
             * @brief Handle CREATE_OBJECT command - initialize OTA session
             * @param writer Response writer callback for sending response
             * @param payload CREATE_OBJECT payload with object type and firmware size
             */
            void doCreateObject(ResponseWriterFn writer, const protocol::create_object_payload_t& payload);

            /**
             * @brief Handle SET_RECEIPT_NOTIFICATION command - configure PRN
             * @param writer Response writer callback for sending response
             * @param payload SET_PRN payload with packet count between notifications
             */
            void doSetPrn(ResponseWriterFn writer, const protocol::set_prn_payload_t& payload);

            /**
             * @brief Handle CALCULATE_CRC command - return current offset and CRC32
             * @param writer Response writer callback for sending CRC response
             */
            void doCalculateCrc(ResponseWriterFn writer);

            /**
             * @brief Handle EXECUTE_OBJECT command - finalize OTA and reboot
             * @param writer Response writer callback for sending final response
             */
            void doExecuteObject(ResponseWriterFn writer);

            /**
             * @brief Handle SELECT_OBJECT command - query OTA session state
             * @param writer Response writer callback for sending state response
             * @param payload SELECT_OBJECT payload with object type to query
             */
            void doSelectObject(ResponseWriterFn writer, const protocol::select_object_payload_t& payload);

            /**
             * @brief Handle binary command dispatch errors
             * @param writer Response writer callback for sending error response
             * @param opcode Failed opcode
             * @param error Dispatch error type (unknown_opcode, payload_too_small, etc.)
             */
            void doDispatchError(ResponseWriterFn writer, uint8_t opcode, blex_binary_command::DispatchError error);
        }  // namespace dfu
    }  // namespace detail
}  // namespace ota

#endif
