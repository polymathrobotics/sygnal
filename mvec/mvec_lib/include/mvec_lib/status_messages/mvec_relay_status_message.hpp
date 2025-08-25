// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_
#define MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/core/j1939_id.hpp"
#include "mvec_lib/core/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

/// @brief Constants for parsing relay status messages
namespace MvecRelayStatusConstants
{
/// @brief Number of bits used to encode each relay status
inline constexpr uint8_t BITS_PER_RELAY_STATUS = 4;
/// @brief Starting byte position of relay data in CAN frame
inline constexpr uint8_t START_BYTE = 1;
}  // namespace MvecRelayStatusConstants

/// @brief Possible status values for MVEC relays
enum class MvecRelayStatus : uint8_t
{
  OKAY = 0x00,                            ///< Relay is functioning normally
  COIL_OPEN = 0x01,                       ///< Relay coil circuit is open
  COIL_SHORTED_OR_DRIVER_FAILED = 0x02,   ///< Relay coil is shorted or driver failed
  NO_CONTACT_OPEN = 0x03,                 ///< Normally-open contact is stuck open
  NC_CONTACT_OPEN = 0x04,                 ///< Normally-closed contact is stuck open
  COIL_NOT_RECEIVING_POWER = 0x05,        ///< Relay coil is not receiving power
  NO_CONTACT_SHORTED = 0x06,              ///< Normally-open contact is shorted
  NC_CONTACT_SHORTED = 0x07,              ///< Normally-closed contact is shorted
  HIGH_SIDE_DRIVER_FAULT = 0x0B,          ///< High-side driver fault detected
  HIGH_SIDE_OPEN_LOAD = 0x0C,             ///< High-side output has open load
  HIGH_SIDE_OVER_VOLTAGE = 0x0D,          ///< High-side output over-voltage condition
  RELAY_LOCATION_NOT_USED = 0x0F          ///< Relay location is not populated or used
};

/// @brief Parser and container for MVEC relay status broadcast messages
/// Provides diagnostic status information for all relays in the MVEC device
class MvecRelayStatusMessage
{
public:
  /// @brief Constructor
  /// @param source_address Expected source address for messages
  /// @param pgn_base_value Base PGN value for message identification
  MvecRelayStatusMessage(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t pgn_base_value = MvecProtocol::DEFAULT_PGN_BASE_VALUE);

  /// @brief Parse CAN frame for relay status data
  /// @param frame CAN frame to parse
  /// @return true if frame was successfully parsed
  bool parse(const socketcan::CanFrame & frame);

  /// @brief Get diagnostic status of specific relay
  /// @param relay_id Relay ID (0-11)
  /// @return Current diagnostic status of the specified relay
  MvecRelayStatus get_relay_status(uint8_t relay_id) const;

  /// @brief Check if parsed data is valid
  /// @return true if message has been successfully parsed
  bool is_valid() const
  {
    return is_valid_;
  }

private:
  /// @brief Expected J1939 message ID for filtering
  J1939_ID expected_id_;
  /// @brief Status array for all relays (0-11)
  std::array<MvecRelayStatus, MvecHardware::MAX_NUMBER_RELAYS> relay_statuses_;
  /// @brief Validity flag for parsed data
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_
