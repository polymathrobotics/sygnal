// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_fuse_status_message.hpp"

namespace polymath::sygnal
{

MvecFuseStatusMessage::MvecFuseStatusMessage(uint8_t source_address, uint8_t pgn_base_value)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::BROADCAST_PDU,
    0x01 + pgn_base_value,
    source_address)
, is_valid_(false)
{
  fuse_statuses_.fill(MvecFuseStatus::NOT_USED);
}

bool MvecFuseStatusMessage::parse(const socketcan::CanFrame & frame)
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

  constexpr uint8_t len_data_bits = MvecFuseConstants::MAX_FUSES * 2;

  auto data = unpackData<uint64_t>(raw_data, 1 * sizeof(unsigned char), len_data_bits);

  for (size_t i = 0; i < MvecFuseConstants::MAX_FUSES; i++) {
    uint8_t raw_status = (data >> (i * 2)) & 0x03;
    fuse_statuses_[i] = static_cast<MvecFuseStatus>(raw_status);
  }

  is_valid_ = true;
  return true;
}

MvecFuseStatus MvecFuseStatusMessage::get_fuse_status(uint8_t fuse_id) const
{
  if (fuse_id >= MvecFuseConstants::MAX_FUSES) {
    return MvecFuseStatus::NOT_USED;
  }
  return fuse_statuses_[fuse_id];
}

}  // namespace polymath::sygnal
