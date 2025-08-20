// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_
#define MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/can_bitwork.hpp"
#include "mvec_lib/j1939_id.hpp"
#include "mvec_lib/mvec_constants.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecRelayConstants
{
inline constexpr int MAX_RELAYS = 12;
inline constexpr int MAX_HIGH_SIDE_OUTPUTS = 1;
inline constexpr uint8_t RELAY_STATUS_DATA_START_BYTE = 1;
inline constexpr uint8_t BITS_PER_RELAY_STATUS = 4;
inline constexpr uint8_t RELAY_STATUS_MASK = 0x0F;
inline constexpr uint8_t MAX_RELAY_STATE_VALUE = 0x01;
inline constexpr uint8_t MAX_HIGH_SIDE_STATE_VALUE = 0x01;
}  // namespace MvecRelayConstants

enum class MvecRelayStatus : uint8_t
{
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

class MvecRelayStatusMessage
{
public:
  MvecRelayStatusMessage(
    uint8_t source_address = MvecProtocol::DEFAULT_SOURCE_ADDRESS,
    uint8_t pgn_base_value = MvecProtocol::DEFAULT_PGN_BASE_VALUE);

  bool parse(const socketcan::CanFrame & frame);

  MvecRelayStatus get_relay_status(uint8_t relay_id) const;

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  J1939_ID expected_id_;
  std::array<MvecRelayStatus, MvecRelayConstants::MAX_RELAYS> relay_statuses_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_STATUS_MESSAGE_HPP_
