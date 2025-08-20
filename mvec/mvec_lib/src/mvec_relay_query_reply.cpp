// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_relay_query_reply.hpp"

namespace polymath::sygnal
{

MvecRelayQueryReply::MvecRelayQueryReply()
: high_side_output_state_(false)
, high_side_output_default_(false)
, is_valid_(false)
{
  relay_states_.fill(false);
  relay_defaults_.fill(false);
}

bool MvecRelayQueryReply::parse(const socketcan::CanFrame & frame)
{
  if (frame.get_id_type() != socketcan::IdType::EXTENDED || frame.get_frame_type() != socketcan::FrameType::DATA) {
    is_valid_ = false;
    return false;
  }

  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Check if this is a relay query response
  if (raw_data[0] != MvecRelayQueryConstants::RELAY_QUERY_RESPONSE_MESSAGE_ID) {
    is_valid_ = false;
    return false;
  }

  // Parse relay states from bytes 2&3
  auto data = unpackData<uint16_t>(
    raw_data, MvecRelayQueryConstants::RELAY_REPLY_STATE_START_BYTE * sizeof(unsigned char), 12 + 1);
  for (size_t i = 0; i < 12; ++i) {
    relay_states_[i] = ((data >> i) & 0x01);
  }
  high_side_output_state_ = ((data >> 12) & 0x01);

  // Parse relay defaults from bytes 4&5
  data = unpackData<uint16_t>(
    raw_data, MvecRelayQueryConstants::RELAY_REPLY_DEFAULT_START_BYTE * sizeof(unsigned char), 12 + 1);
  for (size_t i = 0; i < 12; ++i) {
    relay_defaults_[i] = ((data >> i) & 0x01);
  }
  high_side_output_default_ = ((data >> 12) & 0x01);
  is_valid_ = true;

  return true;
}

bool MvecRelayQueryReply::get_relay_state(uint8_t relay_id) const
{
  if (relay_id >= 12) {
    return false;
  }
  return relay_states_[relay_id];
}

bool MvecRelayQueryReply::get_relay_default(uint8_t relay_id) const
{
  if (relay_id >= 12) {
    return false;
  }
  return relay_defaults_[relay_id];
}

}  // namespace polymath::sygnal
