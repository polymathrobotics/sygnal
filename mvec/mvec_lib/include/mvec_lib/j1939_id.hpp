#ifndef MVEC_LIB__J1939_ID_HPP
#define MVEC_LIB__J1939_ID_HPP

#include <cstdint>
#include <linux/can.h>  // For canid_t

/// @brief Generic bit field definition for extracting/inserting fields in 32-bit values
/// @details Provides compile-time constant operations for bit field manipulation
struct uint32_bit_field {
    uint8_t offset;  ///< Bit position of the field's LSB (0-31)
    uint8_t bits;    ///< Number of bits in the field (1-32)

    /// @brief Generate a bitmask for this field
    /// @return Mask with 1s in the field position, 0s elsewhere
    constexpr uint32_t mask() const {
        return ((1ULL << bits) - 1) << offset;
    }

    /// @brief Extract field value from a 32-bit word
    /// @param value The 32-bit value to extract from
    /// @return The extracted field value, right-shifted to position 0
    constexpr uint32_t extract(uint32_t value) const {
        return (value & mask()) >> offset;
    }

    /// @brief Insert a field value into the correct bit position
    /// @param field_value The value to insert (will be masked to field size)
    /// @return The field value shifted to the correct position with appropriate masking
    constexpr uint32_t insert(uint32_t field_value) const {
        return (field_value & ((1ULL << bits) - 1)) << offset;
    }
};

/// @namespace j1939_fields
/// @brief J1939 CAN identifier bit field definitions per SAE J1939-21
/// @details Defines the structure of the 29-bit extended CAN identifier used in J1939
namespace j1939_fields {
    /// @brief Priority field (3 bits, position 26-28)
    /// @details 0 = highest priority, 7 = lowest priority
    constexpr uint32_bit_field priority       = {26, 3};

    /// @brief Reserved bit (1 bit, position 25)
    /// @details Usually 0, reserved for future use by SAE
    constexpr uint32_bit_field reserved       = {25, 1};

    /// @brief Data Page selector (1 bit, position 24)
    /// @details 0 = Page 0, 1 = Page 1 (selects between two pages of parameter groups)
    constexpr uint32_bit_field data_page      = {24, 1};

    /// @brief PDU Format field (8 bits, position 16-23)
    /// @details Determines PDU type: <240 = PDU1 (peer-to-peer), >=240 = PDU2 (broadcast)
    constexpr uint32_bit_field pdu_format     = {16, 8};

    /// @brief PDU Specific field (8 bits, position 8-15)
    /// @details For PDU1: destination address, For PDU2: group extension
    constexpr uint32_bit_field pdu_specific   = {8,  8};

    /// @brief Source Address field (8 bits, position 0-7)
    /// @details Address of the transmitting ECU (0-253 normal, 254 null, 255 global)
    constexpr uint32_bit_field source_address = {0,  8};
}

/// @class J1939_ID
/// @brief SAE J1939 CAN identifier wrapper class
/// @details Provides structured access to J1939 29-bit extended CAN identifiers
///          with convenient methods for PGN handling and PDU type detection
class J1939_ID {
public:
    /// @brief Construct a J1939 ID with specified field values
    /// @param priority Message priority (0-7, default 6 for normal messages)
    /// @param data_page Data page selector (0-1, default 0)
    /// @param pdu_format PDU format value (0-255, determines PDU1 vs PDU2)
    /// @param pdu_specific PDU specific value (destination address or group extension)
    /// @param source_address Transmitting node address (0-255)
    /// @param reserved Reserved bit value (typically 0)
    J1939_ID(uint8_t priority = 6,
             uint8_t data_page = 0,
             uint8_t pdu_format = 0,
             uint8_t pdu_specific = 0,
             uint8_t source_address = 0,
             uint8_t reserved = 0)
        : can_id(CAN_EFF_FLAG) {  // Set extended frame flag
        set_priority(priority);
        set_data_page(data_page);
        set_pdu_format(pdu_format);
        set_pdu_specific(pdu_specific);
        set_source_address(source_address);
        set_reserved(reserved);
    }

    /// @brief Construct from a raw CAN ID
    /// @param id Raw CAN identifier (will force extended frame flag)
    explicit J1939_ID(canid_t id) : can_id(id | CAN_EFF_FLAG) {}

    /// @brief Get message priority
    /// @return Priority value (0-7, where 0 is highest priority)
    uint8_t get_priority() const       { return get_field(j1939_fields::priority); }

    /// @brief Get reserved bit
    /// @return Reserved bit value (typically 0)
    uint8_t get_reserved() const       { return get_field(j1939_fields::reserved); }

    /// @brief Get data page selector
    /// @return Data page (0 or 1)
    uint8_t get_data_page() const      { return get_field(j1939_fields::data_page); }

    /// @brief Get PDU format field
    /// @return PDU format value (0-255)
    uint8_t get_pdu_format() const     { return get_field(j1939_fields::pdu_format); }

    /// @brief Get PDU specific field
    /// @return PDU specific value (destination address for PDU1, group extension for PDU2)
    uint8_t get_pdu_specific() const   { return get_field(j1939_fields::pdu_specific); }

    /// @brief Get source address
    /// @return Source node address (0-255)
    uint8_t get_source_address() const { return get_field(j1939_fields::source_address); }

    /// @brief Set message priority
    /// @param priority_value Priority (0-7, where 0 is highest)
    void set_priority(uint8_t priority_value)             { set_field(j1939_fields::priority, priority_value); }

    /// @brief Set reserved bit
    /// @param reserved_bit Reserved bit value (should be 0)
    void set_reserved(uint8_t reserved_bit)               { set_field(j1939_fields::reserved, reserved_bit); }

    /// @brief Set data page selector
    /// @param page_number Data page (0 or 1)
    void set_data_page(uint8_t page_number)               { set_field(j1939_fields::data_page, page_number); }

    /// @brief Set PDU format field
    /// @param format_value PDU format (0-255, <240 for PDU1, >=240 for PDU2)
    void set_pdu_format(uint8_t format_value)             { set_field(j1939_fields::pdu_format, format_value); }

    /// @brief Set PDU specific field
    /// @param specific_value PDU specific (destination for PDU1, group extension for PDU2)
    void set_pdu_specific(uint8_t specific_value)         { set_field(j1939_fields::pdu_specific, specific_value); }

    /// @brief Set source address
    /// @param source_addr Source node address (0-255)
    void set_source_address(uint8_t source_addr)          { set_field(j1939_fields::source_address, source_addr); }

    /// @brief Get complete CAN ID including flags
    /// @return CAN ID with extended frame flag set
    canid_t get_can_id() const { return can_id; }

    /// @brief Set CAN ID from raw value
    /// @param new_id New CAN identifier (extended flag will be forced on)
    void set_can_id(canid_t new_id) { can_id = new_id | CAN_EFF_FLAG; }

    /// @brief Get 29-bit identifier without CAN flags
    /// @return Pure 29-bit J1939 identifier value
    uint32_t get_raw_id() const { return can_id & CAN_EFF_MASK; }

    /// @brief Check if this is a PDU1 (peer-to-peer) format message
    /// @return True if PDU format < 240
    bool is_pdu1() const { return get_pdu_format() < 240; }

    /// @brief Check if this is a PDU2 (broadcast) format message
    /// @return True if PDU format >= 240
    bool is_pdu2() const { return get_pdu_format() >= 240; }

    /// @brief Get destination address for PDU1 messages
    /// @return Destination address if PDU1, 0xFF (global) if PDU2
    uint8_t get_destination_address() const {
        return is_pdu1() ? get_pdu_specific() : 0xFF;
    }

    /// @brief Set destination address for PDU1 messages
    /// @param dest_addr Destination node address (only affects PDU1 messages)
    /// @note No effect if message is PDU2 format
    void set_destination_address(uint8_t dest_addr) {
        if (is_pdu1()) {
            set_pdu_specific(dest_addr);
        }
    }

    /// @brief Get group extension for PDU2 messages
    /// @return Group extension if PDU2, 0 if PDU1
    uint8_t get_group_extension() const {
        return is_pdu2() ? get_pdu_specific() : 0;
    }

    /// @brief Set group extension for PDU2 messages
    /// @param group_ext Group extension value (only affects PDU2 messages)
    /// @note No effect if message is PDU1 format
    void set_group_extension(uint8_t group_ext) {
        if (is_pdu2()) {
            set_pdu_specific(group_ext);
        }
    }

    /// @brief Get Parameter Group Number (PGN)
    /// @return 18-bit PGN value
    /// @details PGN consists of Reserved + Data Page + PDU Format + (PDU Specific if PDU2)
    ///          For PDU1, PDU Specific is not part of PGN as it contains destination address
    uint32_t get_pgn() const {
        uint32_t pgn = (get_reserved() << 17) |
                       (get_data_page() << 16) |
                       (get_pdu_format() << 8);

        if (is_pdu2()) {
            pgn |= get_pdu_specific();
        }

        return pgn;
    }

    /// @brief Equality comparison based on 29-bit ID only
    /// @param other Other J1939_ID to compare with
    /// @return True if 29-bit identifiers match (ignoring flags)
    bool operator==(const J1939_ID& other) const { return get_raw_id() == other.get_raw_id(); }

    /// @brief Inequality comparison based on 29-bit ID only
    /// @param other Other J1939_ID to compare with
    /// @return True if 29-bit identifiers differ
    bool operator!=(const J1939_ID& other) const { return get_raw_id() != other.get_raw_id(); }

    /// @brief Less-than comparison for ordering (e.g., in maps)
    /// @param other Other J1939_ID to compare with
    /// @return True if this ID is numerically less than other
    bool operator<(const J1939_ID& other) const { return get_raw_id() < other.get_raw_id(); }

private:
    canid_t can_id;  ///< Internal storage of CAN ID with flags

    /// @brief Generic field extraction from CAN ID
    /// @param field Field definition specifying bit position and width
    /// @return Extracted field value
    /// @note Assumes CAN_EFF_FLAG is always set in can_id
    uint32_t get_field(const uint32_bit_field& field) const {
        return field.extract(can_id);
    }

    /// @brief Generic field insertion into CAN ID
    /// @param field Field definition specifying bit position and width
    /// @param new_value Value to insert (will be masked to field width)
    /// @note Preserves CAN_EFF_FLAG and other fields
    void set_field(const uint32_bit_field& field, uint32_t new_value) {
        can_id = (can_id & ~field.mask()) | field.insert(new_value);
    }
};

// Static assertions to verify field definitions at compile time
static_assert(j1939_fields::priority.mask() == 0x1C000000);
static_assert(j1939_fields::reserved.mask() == 0x02000000);
static_assert(j1939_fields::data_page.mask() == 0x01000000);
static_assert(j1939_fields::pdu_format.mask() == 0x00FF0000);
static_assert(j1939_fields::pdu_specific.mask() == 0x0000FF00);
static_assert(j1939_fields::source_address.mask() == 0x000000FF);

#endif // MVEC_LIB__J1939_ID_HPP
