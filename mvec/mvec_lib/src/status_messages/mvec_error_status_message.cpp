// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/status_messages/mvec_error_status_message.hpp"

namespace polymath::sygnal
{

MvecErrorStatusMessage::MvecErrorStatusMessage(uint8_t source_address, uint8_t pgn_base_value)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::STATUS_PDU,
    0x03 + pgn_base_value,
    source_address)
, grid_address_(0)
, error_bits_(0)
, is_valid_(false)
{}

bool MvecErrorStatusMessage::parse(const socketcan::CanFrame & frame)
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

  grid_address_ = raw_data[0];

  error_bits_ = unpackData<uint16_t>(
    raw_data, MvecErrorStatusConstants::START_BYTE * CHAR_BIT, MvecErrorStatusConstants::NUM_ERROR_BITS);

  is_valid_ = true;
  return true;
}

bool MvecErrorStatusMessage::has_error(MvecErrorType error_type) const
{
  return (error_bits_ & static_cast<uint16_t>(error_type)) != 0;
}

}  // namespace polymath::sygnal
