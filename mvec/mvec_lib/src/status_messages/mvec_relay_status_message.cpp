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

#include "mvec_lib/status_messages/mvec_relay_status_message.hpp"

namespace polymath::sygnal
{

MvecRelayStatusMessage::MvecRelayStatusMessage(uint8_t source_address, uint8_t pgn_base_value)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::STATUS_PDU,
    0x01 + pgn_base_value,
    source_address)
, is_valid_(false)
{
  relay_statuses_.fill(MvecRelayStatus::RELAY_LOCATION_NOT_USED);
}

bool MvecRelayStatusMessage::parse(const socketcan::CanFrame & frame)
{
  /// TODO: (Zeerek) use frame timestamp once supported to set validity
  if (frame.get_id_type() != socketcan::IdType::EXTENDED || frame.get_frame_type() != socketcan::FrameType::DATA) {
    return false;
  }

  // Check if this frame matches our expected J1939 ID
  J1939_ID frame_id(frame.get_id());
  if (!(frame_id == expected_id_)) {
    return false;
  }

  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  constexpr uint8_t len_data_bits = MvecHardware::MAX_NUMBER_RELAYS * MvecRelayStatusConstants::BITS_PER_RELAY_STATUS;

  auto data = unpackData<uint64_t>(raw_data, MvecRelayStatusConstants::START_BYTE * CHAR_BIT, len_data_bits);

  for (size_t i = 0; i < MvecHardware::MAX_NUMBER_RELAYS; i++) {
    uint8_t raw_status = (data >> (i * MvecRelayStatusConstants::BITS_PER_RELAY_STATUS)) & 0x0F;
    relay_statuses_[i] = static_cast<MvecRelayStatus>(raw_status);
  }

  is_valid_ = true;
  return true;
}

MvecRelayStatus MvecRelayStatusMessage::get_relay_status(uint8_t relay_id) const
{
  if (relay_id >= MvecHardware::MAX_NUMBER_RELAYS) {
    return MvecRelayStatus::RELAY_LOCATION_NOT_USED;
  }
  return relay_statuses_[relay_id];
}

}  // namespace polymath::sygnal
