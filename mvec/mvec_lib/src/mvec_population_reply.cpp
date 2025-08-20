// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#include "mvec_lib/mvec_population_reply.hpp"

namespace polymath::sygnal
{

MvecPopulationReply::MvecPopulationReply()
: high_side_output_population_(false)
, is_valid_(false)
{
  fuse_population_.fill(false);
  relay_population_.fill(false);
}

bool MvecPopulationReply::parse(const socketcan::CanFrame & frame)
{
  if (frame.get_id_type() != socketcan::IdType::EXTENDED || frame.get_frame_type() != socketcan::FrameType::DATA) {
    is_valid_ = false;
    return false;
  }

  std::array<unsigned char, CAN_MAX_DLC> raw_data = frame.get_data();

  // Check if this is a population response
  if (raw_data[0] != MvecPopulationConstants::POPULATION_RESPONSE_MESSAGE_ID) {
    is_valid_ = false;
    return false;
  }

  // Parse fuse population from bytes 2-4
  auto data =
    unpackData<uint32_t>(raw_data, MvecPopulationConstants::POPULATION_FUSE_START_BYTE * sizeof(unsigned char), 24);
  for (size_t i = 0; i < 24; ++i) {
    fuse_population_[i] = ((data >> i) & 0x01);
  }

  // Parse relay population from bytes 5&6
  data = unpackData<uint32_t>(
    raw_data, MvecPopulationConstants::POPULATION_RELAY_START_BYTE * sizeof(unsigned char), 12 + 1);
  for (size_t i = 0; i < 12; ++i) {
    relay_population_[i] = ((data >> i) & 0x01);
  }
  high_side_output_population_ = ((data >> 12) & 0x01);
  is_valid_ = true;

  return true;
}

bool MvecPopulationReply::get_fuse_population(uint8_t fuse_id) const
{
  if (fuse_id >= 24) {
    return false;
  }
  return fuse_population_[fuse_id];
}

bool MvecPopulationReply::get_relay_population(uint8_t relay_id) const
{
  if (relay_id >= 12) {
    return false;
  }
  return relay_population_[relay_id];
}

}  // namespace polymath::sygnal
