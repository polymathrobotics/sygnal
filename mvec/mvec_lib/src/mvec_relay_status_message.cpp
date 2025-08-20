// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_relay_status_message.hpp"

namespace polymath::sygnal
{

MvecRelayStatusMessage::MvecRelayStatusMessage(uint8_t source_address, uint8_t pgn_base_value)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::BROADCAST_PDU,
    0x02 + pgn_base_value,
    source_address)
, is_valid_(false)
{
  relay_statuses_.fill(MvecRelayStatus::RELAY_LOCATION_NOT_USED);
}

bool MvecRelayStatusMessage::parse(const socketcan::CanFrame & frame)
{
  if (frame.get_id_type() != socketcan::IdType::EXTENDED || frame.get_frame_type() != socketcan::FrameType::DATA) {
    is_valid_ = false;
    return false;
  }

  // Check if this frame matches our expected J1939 ID
  J1939_ID frame_id(frame.get_id());
  if (!(frame_id == expected_id_)) {
    is_valid_ = false;
    return false;
  }

  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  constexpr uint8_t len_data_bits = MvecRelayConstants::MAX_RELAYS * 4;

  auto data = unpackData<uint64_t>(raw_data, 1 * sizeof(unsigned char), len_data_bits);

  for (size_t i = 0; i < MvecRelayConstants::MAX_RELAYS; i++) {
    uint8_t raw_status = (data >> (i * 4)) & 0x0F;
    relay_statuses_[i] = static_cast<MvecRelayStatus>(raw_status);
  }

  is_valid_ = true;
  return true;
}

MvecRelayStatus MvecRelayStatusMessage::get_relay_status(uint8_t relay_id) const
{
  if (relay_id >= MvecRelayConstants::MAX_RELAYS) {
    return MvecRelayStatus::RELAY_LOCATION_NOT_USED;
  }
  return relay_statuses_[relay_id];
}

}  // namespace polymath::sygnal
