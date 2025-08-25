// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_
#define MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_

#include <linux/can.h>
#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/responses/mvec_response_base.hpp"

namespace polymath::sygnal
{

namespace MvecRelayQueryConstants
{
inline constexpr uint8_t RELAY_REPLY_STATE_START_BYTE = 2;
inline constexpr uint8_t RELAY_REPLY_DEFAULT_START_BYTE = 4;
inline constexpr uint8_t RELAY_QUERY_RESPONSE_MESSAGE_ID = 0x96;
}  // namespace MvecRelayQueryConstants

/// @brief Parser and container for MVEC relay query responses
/// Contains current relay states and default power-on states
class MvecRelayQueryReply : public MvecResponseBase
{
public:
  /// @brief Constructor
  /// @param source_address Expected source address for messages
  /// @param my_address Self address for message filtering
  MvecRelayQueryReply(uint8_t source_address, uint8_t my_address);

  /// @brief Get current state of specific relay
  /// @param relay_id Relay ID (0-11)
  /// @return true if relay is currently on
  bool get_relay_state(uint8_t relay_id) const;

  /// @brief Get current state of high side output
  /// @return true if high side output is currently on
  bool get_high_side_output_state() const
  {
    return high_side_output_state_;
  }

  /// @brief Get default power-on state of specific relay
  /// @param relay_id Relay ID (0-11)
  /// @return true if relay defaults to on at power-up
  bool get_relay_default(uint8_t relay_id) const;

  /// @brief Get default power-on state of high side output
  /// @return true if high side output defaults to on at power-up
  bool get_high_side_output_default() const
  {
    return high_side_output_default_;
  }

protected:
  bool parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data) override;

private:
  std::array<bool, 12> relay_states_;
  bool high_side_output_state_;
  std::array<bool, 12> relay_defaults_;
  bool high_side_output_default_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_
