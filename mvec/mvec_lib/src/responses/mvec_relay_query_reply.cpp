// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mvec_lib/responses/mvec_relay_query_reply.hpp"

#include "mvec_lib/core/mvec_constants.hpp"

namespace polymath::sygnal
{

MvecRelayQueryReply::MvecRelayQueryReply(uint8_t source_address, uint8_t my_address)
: MvecResponseBase(MvecRelayQueryConstants::RELAY_QUERY_RESPONSE_MESSAGE_ID, source_address, my_address)
, high_side_output_state_(false)
, high_side_output_default_(false)
{
  relay_states_.fill(false);
  relay_defaults_.fill(false);
}

bool MvecRelayQueryReply::parse_message_data(const std::array<unsigned char, CAN_MAX_DLC> & data)
{
  // Parse relay states from bytes 2&3
  auto state_data = unpackData<uint16_t>(
    data,
    MvecRelayQueryConstants::RELAY_REPLY_STATE_START_BYTE * CHAR_BIT,
    MvecHardware::MAX_NUMBER_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_RELAYS; ++i) {
    relay_states_[i] = ((state_data >> i) & 0x01);
  }
  high_side_output_state_ = ((state_data >> MvecHardware::MAX_NUMBER_RELAYS) & 0x01);

  // Parse relay defaults from bytes 4&5
  auto default_data = unpackData<uint16_t>(
    data,
    MvecRelayQueryConstants::RELAY_REPLY_DEFAULT_START_BYTE * CHAR_BIT,
    MvecHardware::MAX_NUMBER_RELAYS + MvecHardware::MAX_HIGH_SIDE_OUTPUTS);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_RELAYS; ++i) {
    relay_defaults_[i] = ((default_data >> i) & 0x01);
  }
  high_side_output_default_ = ((default_data >> MvecHardware::MAX_NUMBER_RELAYS) & 0x01);

  return true;
}

bool MvecRelayQueryReply::get_relay_state(uint8_t relay_id) const
{
  if (relay_id >= MvecHardware::MAX_NUMBER_RELAYS) {
    return false;
  }
  return relay_states_[relay_id];
}

bool MvecRelayQueryReply::get_relay_default(uint8_t relay_id) const
{
  if (relay_id >= MvecHardware::MAX_NUMBER_RELAYS) {
    return false;
  }
  return relay_defaults_[relay_id];
}

}  // namespace polymath::sygnal
