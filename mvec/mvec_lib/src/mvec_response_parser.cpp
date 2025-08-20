// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_response_parser.hpp"

namespace polymath::sygnal
{

MvecResponseParser::MvecResponseParser(uint8_t source_address, uint8_t my_address)
: expected_id_(
    MvecProtocol::DEFAULT_PRIORITY,
    MvecProtocol::DEFAULT_DATA_PAGE,
    MvecProtocol::SPECIFIC_PDU,
    my_address,
    source_address)
, message_id_(0)
, is_valid_(false)
{}

bool MvecResponseParser::parse(const socketcan::CanFrame & frame)
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
  message_id_ = raw_data[0];

  // Try each reply type - they will validate their own message ID
  if (relay_command_reply_.parse(frame)) {
    is_valid_ = true;
    return true;
  }

  if (relay_query_reply_.parse(frame)) {
    is_valid_ = true;
    return true;
  }

  if (population_reply_.parse(frame)) {
    is_valid_ = true;
    return true;
  }

  is_valid_ = false;
  return false;
}

}  // namespace polymath::sygnal
