// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/responses/mvec_relay_command_reply.hpp"

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::sygnal
{

MvecRelayCommandReply::MvecRelayCommandReply(
  uint8_t source_address,
  uint8_t my_address)
: MvecResponseBase(MvecRelayCommandConstants::RESPONSE_MESSAGE_ID, source_address, my_address)
, command_msg_id_(0)
, success_(0)
, error_(0)
, high_side_output_result_(false)
{
  relay_results_.fill(false);
}

bool MvecRelayCommandReply::parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data)
{
  command_msg_id_ = data[1];
  success_ = data[MvecRelayCommandConstants::COMMAND_REPLY_SUCCESS_BYTE];
  error_ = data[MvecRelayCommandConstants::COMMAND_REPLY_ERROR_BYTE];

  auto relay_data = unpackData<uint16_t>(
    data,
    MvecRelayCommandConstants::COMMAND_REPLY_DATA_START_BYTE * CHAR_BIT, 
    MvecHardware::MAX_NUMBER_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_RELAYS; ++i) {
    relay_results_[i] = ((relay_data >> i) & 0x01);
  }
  high_side_output_result_ = ((relay_data >> MvecHardware::MAX_NUMBER_RELAYS) & 0x01);

  return true;
}

bool MvecRelayCommandReply::get_relay_result(uint8_t relay_id) const
{
  if (relay_id >= MvecHardware::MAX_NUMBER_RELAYS) {
    return false;
  }
  return relay_results_[relay_id];
}

}  // namespace polymath::sygnal
