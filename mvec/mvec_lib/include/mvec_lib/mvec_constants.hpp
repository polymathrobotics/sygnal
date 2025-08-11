#ifndef MVEC_LIB__MVEC_CONSTANTS_HPP_
#define MVEC_LIB__MVEC_CONSTANTS_HPP_

#include <stdint.h>

namespace polymath::sygnal
{

struct MvecHardware {
    static constexpr int MAX_RELAYS = 12;
    static constexpr int MAX_HIGH_SIDE_OUTPUTS = 1;
    static constexpr int MAX_FUSES = 24;
    static constexpr uint8_t NUM_ERROR_BITS = 13;
};

struct MvecProtocol {
    static constexpr uint8_t BROADCAST_PDU = 0xFF;
    static constexpr uint8_t SPECIFIC_PDU = 0xEF;
    static constexpr uint8_t DEFAULT_PRIORITY = 6;
    static constexpr uint8_t DEFAULT_DATA_PAGE = 0;
    static constexpr uint8_t RESERVED_BIT = 0;
    static constexpr uint8_t DEFAULT_SOURCE_ADDRESS = 0xB0;
    static constexpr uint8_t DEFAULT_PGN_BASE_VALUE = 0xA0;
    static constexpr uint8_t DEFAULT_SELF_ADDRESS = 0x00;
};

struct MvecMessageIds {
    static constexpr uint8_t RELAY_COMMAND_WITH_FEEDBACK = 0x88;
    static constexpr uint8_t RELAY_COMMAND_NO_FEEDBACK = 0x80;
    static constexpr uint8_t RELAY_STATE_QUERY = 0x96;
    static constexpr uint8_t POPULATION_QUERY = 0x92;
    static constexpr uint8_t RESPONSE = 0x01;
    static constexpr uint8_t POPULATION_RESPONSE = 0x94;
    static constexpr uint8_t RELAY_QUERY_RESPONSE = 0x96;
};

struct MvecMessageStructure {
    static constexpr uint8_t MSG_ID_BYTE = 0;
    static constexpr uint8_t GRID_ID_BYTE = 1;
    static constexpr uint8_t RELAY_DATA_START_BIT = 16;
    static constexpr uint8_t BITS_PER_RELAY = 2;
    static constexpr uint8_t BITS_PER_FUSE = 2;
    static constexpr uint8_t BITS_PER_RELAY_STATUS = 4;
    static constexpr uint8_t HIGH_SIDE_BITS = 2;
};

struct MvecParsingPositions {
    static constexpr uint8_t FUSE_STATUS_DATA_START_BYTE = 1;
    static constexpr uint8_t RELAY_STATUS_DATA_START_BYTE = 1;
    static constexpr uint8_t ERROR_STATUS_DATA_START_BYTE = 1;
    static constexpr uint8_t RELAY_REPLY_STATE_START_BYTE = 2;
    static constexpr uint8_t RELAY_REPLY_DEFAULT_START_BYTE = 4;
    static constexpr uint8_t POPULATION_FUSE_START_BYTE = 2;
    static constexpr uint8_t POPULATION_RELAY_START_BYTE = 5;
    static constexpr uint8_t COMMAND_REPLY_SUCCESS_BYTE = 2;
    static constexpr uint8_t COMMAND_REPLY_ERROR_BYTE = 3;
    static constexpr uint8_t COMMAND_REPLY_DATA_START_BYTE = 4;
};

struct MvecValueLimits {
    static constexpr uint8_t MAX_RELAY_STATE_VALUE = 0x01;
    static constexpr uint8_t MAX_HIGH_SIDE_STATE_VALUE = 0x01;
    static constexpr uint8_t FUSE_STATE_MASK = 0x03;
    static constexpr uint8_t RELAY_STATUS_MASK = 0x0F;
};

// Status value enumerations
enum class MvecFuseStatus : uint8_t {
    NO_FAULT = 0x0,
    BLOWN = 0x1,
    NOT_POWERED = 0x2,
    NOT_USED = 0x3
};

enum class MvecRelayStatus : uint8_t {
    OKAY = 0x00,
    COIL_OPEN = 0x01,
    COIL_SHORTED_OR_DRIVER_FAILED = 0x02,
    NO_CONTACT_OPEN = 0x03,
    NC_CONTACT_OPEN = 0x04,
    COIL_NOT_RECEIVING_POWER = 0x05,
    NO_CONTACT_SHORTED = 0x06,
    NC_CONTACT_SHORTED = 0x07,
    HIGH_SIDE_DRIVER_FAULT = 0x0B,
    HIGH_SIDE_OPEN_LOAD = 0x0C,
    HIGH_SIDE_OVER_VOLTAGE = 0x0D,
    RELAY_LOCATION_NOT_USED = 0x0F
};

// Error status bit masks and positions
struct MvecErrorBits {
    // Byte 1 bits (positions 0-7)
    static constexpr uint16_t INVALID_CONFIG = 0x0001;                    // Bit 1.0
    static constexpr uint16_t GRID_ID_CHANGED = 0x0002;                   // Bit 1.1
    static constexpr uint16_t CAN_ADDRESS_CHANGED = 0x0004;               // Bit 1.2
    static constexpr uint16_t CAN_RX_COMM_ERROR = 0x0008;                 // Bit 1.3
    static constexpr uint16_t CAN_TX_COMM_ERROR = 0x0010;                 // Bit 1.4
    static constexpr uint16_t UNEXPECTED_RESET = 0x0020;                  // Bit 1.5
    static constexpr uint16_t OVER_VOLTAGE = 0x0040;                      // Bit 1.6
    static constexpr uint16_t SPI_ERROR = 0x0080;                         // Bit 1.7
    
    // Byte 2 bits (positions 8-12)
    static constexpr uint16_t SHORT_MESSAGE_RECEIVED = 0x0100;            // Bit 2.0
    static constexpr uint16_t BAD_FLASH_ADDRESS = 0x0200;                 // Bit 2.1
    static constexpr uint16_t INVALID_LENGTH = 0x0400;                    // Bit 2.2
    static constexpr uint16_t CHECKSUM_FAILURE = 0x0800;                  // Bit 2.3
    static constexpr uint16_t FLASH_MISCOMPARE = 0x1000;                  // Bit 2.4
};

enum class MvecErrorType : uint16_t {
    INVALID_CONFIG = MvecErrorBits::INVALID_CONFIG,
    GRID_ID_CHANGED = MvecErrorBits::GRID_ID_CHANGED,
    CAN_ADDRESS_CHANGED = MvecErrorBits::CAN_ADDRESS_CHANGED,
    CAN_RX_COMM_ERROR = MvecErrorBits::CAN_RX_COMM_ERROR,
    CAN_TX_COMM_ERROR = MvecErrorBits::CAN_TX_COMM_ERROR,
    UNEXPECTED_RESET = MvecErrorBits::UNEXPECTED_RESET,
    OVER_VOLTAGE = MvecErrorBits::OVER_VOLTAGE,
    SPI_ERROR = MvecErrorBits::SPI_ERROR,
    SHORT_MESSAGE_RECEIVED = MvecErrorBits::SHORT_MESSAGE_RECEIVED,
    BAD_FLASH_ADDRESS = MvecErrorBits::BAD_FLASH_ADDRESS,
    INVALID_LENGTH = MvecErrorBits::INVALID_LENGTH,
    CHECKSUM_FAILURE = MvecErrorBits::CHECKSUM_FAILURE,
    FLASH_MISCOMPARE = MvecErrorBits::FLASH_MISCOMPARE
};

} // namespace polymath::sygnal

#endif // MVEC_LIB__MVEC_CONSTANTS_HPP_