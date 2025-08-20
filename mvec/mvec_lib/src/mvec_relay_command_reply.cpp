// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_relay_command_reply.hpp"

namespace polymath::sygnal
{

MvecRelayCommandReply::MvecRelayCommandReply()
: command_msg_id_(0)
, success_(0)
, error_(0)
, high_side_output_result_(false)
, is_valid_(false)
{
  relay_results_.fill(false);
}

bool MvecRelayCommandReply::parse(const socketcan::CanFrame & frame)
{
  if (frame.get_id_type() != socketcan::IdType::EXTENDED || frame.get_frame_type() != socketcan::FrameType::DATA) {
    is_valid_ = false;
    return false;
  }

  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Check if this is a relay command response
  if (raw_data[0] != MvecRelayCommandConstants::RESPONSE_MESSAGE_ID) {
    is_valid_ = false;
    return false;
  }

  command_msg_id_ = raw_data[1];
  success_ = raw_data[MvecRelayCommandConstants::COMMAND_REPLY_SUCCESS_BYTE];
  error_ = raw_data[MvecRelayCommandConstants::COMMAND_REPLY_ERROR_BYTE];

  auto data = unpackData<uint16_t>(
    raw_data, MvecRelayCommandConstants::COMMAND_REPLY_DATA_START_BYTE * sizeof(unsigned char), 12 + 1);

  for (size_t i = 0; i < 12; ++i) {
    relay_results_[i] = ((data >> i) & 0x01);
  }
  high_side_output_result_ = ((data >> 12) & 0x01);
  is_valid_ = true;

  return true;
}

bool MvecRelayCommandReply::get_relay_result(uint8_t relay_id) const
{
  if (relay_id >= 12) {
    return false;
  }
  return relay_results_[relay_id];
}

}  // namespace polymath::sygnal
