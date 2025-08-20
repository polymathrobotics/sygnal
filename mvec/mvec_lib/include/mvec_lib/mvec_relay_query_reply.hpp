// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_
#define MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/can_bitwork.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecRelayQueryConstants
{
inline constexpr uint8_t RELAY_REPLY_STATE_START_BYTE = 2;
inline constexpr uint8_t RELAY_REPLY_DEFAULT_START_BYTE = 4;
inline constexpr uint8_t RELAY_QUERY_RESPONSE_MESSAGE_ID = 0x96;
}  // namespace MvecRelayQueryConstants

class MvecRelayQueryReply
{
public:
  MvecRelayQueryReply();

  bool parse(const socketcan::CanFrame & frame);

  bool get_relay_state(uint8_t relay_id) const;

  bool get_high_side_output_state() const
  {
    return high_side_output_state_;
  }

  bool get_relay_default(uint8_t relay_id) const;

  bool get_high_side_output_default() const
  {
    return high_side_output_default_;
  }

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  std::array<bool, 12> relay_states_;
  bool high_side_output_state_;
  std::array<bool, 12> relay_defaults_;
  bool high_side_output_default_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_QUERY_REPLY_HPP_
