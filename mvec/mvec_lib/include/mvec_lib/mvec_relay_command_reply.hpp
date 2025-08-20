// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
#define MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_

#include <stdint.h>

#include <array>

#include "mvec_lib/can_bitwork.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::sygnal
{

namespace MvecRelayCommandConstants
{
inline constexpr uint8_t COMMAND_REPLY_SUCCESS_BYTE = 2;
inline constexpr uint8_t COMMAND_REPLY_ERROR_BYTE = 3;
inline constexpr uint8_t COMMAND_REPLY_DATA_START_BYTE = 4;
inline constexpr uint8_t RESPONSE_MESSAGE_ID = 0x01;
}  // namespace MvecRelayCommandConstants

class MvecRelayCommandReply
{
public:
  MvecRelayCommandReply();

  bool parse(const socketcan::CanFrame & frame);

  uint8_t get_command_msg_id() const
  {
    return command_msg_id_;
  }

  uint8_t get_success() const
  {
    return success_;
  }

  uint8_t get_error() const
  {
    return error_;
  }

  bool get_relay_result(uint8_t relay_id) const;

  bool get_high_side_output_result() const
  {
    return high_side_output_result_;
  }

  bool is_valid() const
  {
    return is_valid_;
  }

private:
  uint8_t command_msg_id_;
  uint8_t success_;
  uint8_t error_;
  std::array<bool, 12> relay_results_;
  bool high_side_output_result_;
  bool is_valid_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
