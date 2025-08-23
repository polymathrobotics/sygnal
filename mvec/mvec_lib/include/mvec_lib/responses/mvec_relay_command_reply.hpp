// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
#define MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_

#include <linux/can.h>
#include <stdint.h>

#include <array>

#include "mvec_lib/core/can_bitwork.hpp"
#include "mvec_lib/responses/mvec_response_base.hpp"

namespace polymath::sygnal
{

namespace MvecRelayCommandConstants
{
inline constexpr uint8_t COMMAND_REPLY_SUCCESS_BYTE = 2;
inline constexpr uint8_t COMMAND_REPLY_ERROR_BYTE = 3;
inline constexpr uint8_t COMMAND_REPLY_DATA_START_BYTE = 4;
inline constexpr uint8_t RESPONSE_MESSAGE_ID = 0x01;
}  // namespace MvecRelayCommandConstants

class MvecRelayCommandReply : public MvecResponseBase
{
public:
  MvecRelayCommandReply(uint8_t source_address, uint8_t my_address);

public:
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

protected:
  bool parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data) override;

private:
  uint8_t command_msg_id_;
  uint8_t success_;
  uint8_t error_;
  std::array<bool, 12> relay_results_;
  bool high_side_output_result_;
};

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__MVEC_RELAY_COMMAND_REPLY_HPP_
